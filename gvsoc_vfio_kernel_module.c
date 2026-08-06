// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal host-side PCI driver for the GVSoC vfio-user DMA bridge.
 *
 * Features:
 * - Binds to the custom PCI endpoint
 * - Maps BAR0 (DMA control BAR)
 * - Allocates one coherent DMA buffer for host <-> device-local transfers
 * - Uses MSI-X completion instead of BAR polling
 * - Exposes a miscdevice /dev/vfio_bridge_dma
 * - Supports mmap() of the coherent buffer into userspace
 * - Supports ioctls to get buffer info and submit a DMA transfer
 *
 * This driver is intentionally minimal and single-device oriented.
 */

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DRV_NAME            "gvsoc_vfio_kernel_module"
#define DEV_NAME            "gvsoc"
#define DMA_BUFFER_SIZE     (1U << 12)
#define DMA_TIMEOUT_MS      5000

/* Replace these with the actual IDs exposed by the bridge. */
#ifndef VFIO_BRIDGE_VENDOR_ID
#define VFIO_BRIDGE_VENDOR_ID 0x1d1d
#endif
#ifndef VFIO_BRIDGE_DEVICE_ID
#define VFIO_BRIDGE_DEVICE_ID 0x0001
#endif

/* BAR0 registers */
#define BAR0_DMA_SRC_ADDR_LO 0x00
#define BAR0_DMA_SRC_ADDR_HI 0x04
#define BAR0_DMA_DST_ADDR_LO 0x08
#define BAR0_DMA_DST_ADDR_HI 0x0c
#define BAR0_DMA_LEN         0x10
#define BAR0_DMA_CTRL        0x14
#define BAR0_DMA_STATUS      0x18
#define BAR0_DMA_ERROR       0x1c
#define BAR0_DMA_MAGIC       0x20
#define BAR0_DMA_DIRECTION   0x24
#define BAR0_ENTRY_POINT     0x28
#define BAR0_FETCH_ENABLE    0x2C

#define DMA_CTRL_START   BIT(0)
#define DMA_CTRL_ABORT   BIT(1)
#define DMA_CTRL_IRQ_EN  BIT(2)

#define DMA_STATUS_BUSY  BIT(0)
#define DMA_STATUS_DONE  BIT(1)
#define DMA_STATUS_ERROR BIT(2)

#define DMA_DIR_HOST_TO_CARD 0
#define DMA_DIR_CARD_TO_HOST 1

struct vfio_bridge_dma_info {
    __u64 dma_addr;
    __u32 size;
    __u32 reserved;
};

struct vfio_bridge_dma_submit {
    __u32 direction;
    __u32 timeout_ms;
    __u64 device_addr; /* device-local memory offset, not a PCI BAR address */
    __u32 len;
    __u32 status;
    __u32 error;
    __u32 reserved;
};

struct vfio_bridge_bar0_write32 {
    __u32 offset;
    __u32 value;
};

#define VFIO_BRIDGE_IOC_MAGIC     'V'
#define VFIO_BRIDGE_IOC_GET_INFO  _IOR(VFIO_BRIDGE_IOC_MAGIC, 0x01, struct vfio_bridge_dma_info)
#define VFIO_BRIDGE_IOC_SUBMIT    _IOWR(VFIO_BRIDGE_IOC_MAGIC, 0x02, struct vfio_bridge_dma_submit)
#define VFIO_BRIDGE_IOC_BAR0_WRITE32 _IOW(VFIO_BRIDGE_IOC_MAGIC, 0x03, struct vfio_bridge_bar0_write32)
#define VFIO_BRIDGE_IOC_WAIT_EOC _IO(VFIO_BRIDGE_IOC_MAGIC, 0x04)

struct vfio_bridge_dev {
    struct pci_dev *pdev;
    void __iomem *bar0;
    resource_size_t bar0_len;

    void *dma_cpu_addr;
    dma_addr_t dma_handle;
    size_t dma_size;

    int irq_vector;
    int irq_eoc;
    struct completion dma_done;
    struct completion eoc_done;
    struct mutex lock;

    u32 last_status;
    u32 last_error;

    struct miscdevice miscdev;
};

static struct pci_device_id vfio_bridge_pci_ids[] = {
    { PCI_DEVICE(VFIO_BRIDGE_VENDOR_ID, VFIO_BRIDGE_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, vfio_bridge_pci_ids);

static inline void vfio_bridge_bar0_write32(struct vfio_bridge_dev *d, u32 off, u32 val)
{
    iowrite32(val, d->bar0 + off);
}

static inline u32 vfio_bridge_bar0_read32(struct vfio_bridge_dev *d, u32 off)
{
    return ioread32(d->bar0 + off);
}

static inline void vfio_bridge_bar0_write64(struct vfio_bridge_dev *d, u32 off_lo, u64 val)
{
    vfio_bridge_bar0_write32(d, off_lo, (u32)(val & 0xffffffffULL));
    vfio_bridge_bar0_write32(d, off_lo + 4, (u32)(val >> 32));
}

static irqreturn_t vfio_bridge_irq_handler(int irq, void *data)
{
    struct vfio_bridge_dev *d = data;

    d->last_status = vfio_bridge_bar0_read32(d, BAR0_DMA_STATUS);
    d->last_error  = vfio_bridge_bar0_read32(d, BAR0_DMA_ERROR);

    complete(&d->dma_done);
    return IRQ_HANDLED;
}

static irqreturn_t vfio_bridge_eoc_handler(int irq, void *data)
{
    struct vfio_bridge_dev *d = data;

    complete(&d->eoc_done);

    return IRQ_HANDLED;
}

static int vfio_bridge_submit_dma(struct vfio_bridge_dev *d,
                                  struct vfio_bridge_dma_submit *req)
{
    unsigned long timeout_jiffies;
    unsigned long wait_ret;
    u32 ctrl;

    if (req->direction != DMA_DIR_HOST_TO_CARD &&
        req->direction != DMA_DIR_CARD_TO_HOST)
        return -EINVAL;

    if (req->len == 0 || req->len > d->dma_size)
        return -EINVAL;

    mutex_lock(&d->lock);

    reinit_completion(&d->dma_done);
    d->last_status = 0;
    d->last_error = 0;

    if (req->direction == DMA_DIR_HOST_TO_CARD) {
        vfio_bridge_bar0_write64(d, BAR0_DMA_SRC_ADDR_LO, (u64)d->dma_handle);
        vfio_bridge_bar0_write64(d, BAR0_DMA_DST_ADDR_LO, req->device_addr);
    } else {
        vfio_bridge_bar0_write64(d, BAR0_DMA_SRC_ADDR_LO, req->device_addr);
        vfio_bridge_bar0_write64(d, BAR0_DMA_DST_ADDR_LO, (u64)d->dma_handle);
    }

    vfio_bridge_bar0_write32(d, BAR0_DMA_LEN, req->len);
    vfio_bridge_bar0_write32(d, BAR0_DMA_DIRECTION, req->direction);

    ctrl = DMA_CTRL_START | DMA_CTRL_IRQ_EN;
    vfio_bridge_bar0_write32(d, BAR0_DMA_CTRL, ctrl);

    timeout_jiffies = msecs_to_jiffies(req->timeout_ms ? req->timeout_ms : DMA_TIMEOUT_MS);
    wait_ret = wait_for_completion_timeout(&d->dma_done, timeout_jiffies);

    if (wait_ret == 0) {
        req->status = vfio_bridge_bar0_read32(d, BAR0_DMA_STATUS);
        req->error  = vfio_bridge_bar0_read32(d, BAR0_DMA_ERROR);
        mutex_unlock(&d->lock);
        return -ETIMEDOUT;
    }

    req->status = d->last_status;
    req->error  = d->last_error;

    mutex_unlock(&d->lock);

    return (req->status & DMA_STATUS_ERROR) ? -EIO : 0;
}

static int vfio_bridge_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *misc = filp->private_data;
    struct vfio_bridge_dev *d = container_of(misc, struct vfio_bridge_dev, miscdev);

    filp->private_data = d;
    return 0;
}

static long vfio_bridge_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct vfio_bridge_dev *d = filp->private_data;

    switch (cmd) {
    case VFIO_BRIDGE_IOC_GET_INFO:
    {
        struct vfio_bridge_dma_info info;

        info.dma_addr = (u64)d->dma_handle;
        info.size = d->dma_size;
        info.reserved = 0;

        if (copy_to_user((void __user *)arg, &info, sizeof(info)))
            return -EFAULT;
        return 0;
    }
    case VFIO_BRIDGE_IOC_SUBMIT:
    {
        struct vfio_bridge_dma_submit req;
        long ret;

        if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
            return -EFAULT;

        ret = vfio_bridge_submit_dma(d, &req);

        if (copy_to_user((void __user *)arg, &req, sizeof(req)))
            return -EFAULT;
        return ret;
    }
    case VFIO_BRIDGE_IOC_BAR0_WRITE32:
    {
        struct vfio_bridge_bar0_write32 req;

        if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
            return -EFAULT;

        switch (req.offset) {
        case BAR0_ENTRY_POINT:
        case BAR0_FETCH_ENABLE:
            break;
        default:
            return -EINVAL;
        }

        mutex_lock(&d->lock);
        vfio_bridge_bar0_write32(d, req.offset, req.value);
        mutex_unlock(&d->lock);
        return 0;
    }
    case VFIO_BRIDGE_IOC_WAIT_EOC:
    {
        long ret;

        reinit_completion(&d->eoc_done);

        ret = wait_for_completion_interruptible(&d->eoc_done);

        return ret;
    }
    default:
        return -ENOTTY;
    }
}

static int vfio_bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct vfio_bridge_dev *d = filp->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;

    if (size > d->dma_size)
        return -EINVAL;

    return dma_mmap_coherent(&d->pdev->dev, vma,
                             d->dma_cpu_addr, d->dma_handle, d->dma_size);
}

static const struct file_operations vfio_bridge_fops = {
    .owner          = THIS_MODULE,
    .open           = vfio_bridge_open,
    .unlocked_ioctl = vfio_bridge_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = vfio_bridge_unlocked_ioctl,
#endif
    .mmap           = vfio_bridge_mmap,
    .llseek         = no_llseek,
};

static int vfio_bridge_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct vfio_bridge_dev *d;
    int ret;
    u32 magic;

    d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
    if (!d)
        return -ENOMEM;

    d->pdev = pdev;
    d->dma_size = DMA_BUFFER_SIZE;
    mutex_init(&d->lock);
    init_completion(&d->dma_done);
    init_completion(&d->eoc_done);

    pci_set_drvdata(pdev, d);

    ret = pcim_enable_device(pdev);
    if (ret)
        return ret;

    pci_set_master(pdev);

    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret)
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (ret)
        return ret;

    ret = pcim_iomap_regions(pdev, BIT(0), DRV_NAME);
    if (ret)
        return ret;

    d->bar0 = pcim_iomap_table(pdev)[0];
    if (!d->bar0)
        return -ENODEV;

    d->bar0_len = pci_resource_len(pdev, 0);

    magic = vfio_bridge_bar0_read32(d, BAR0_DMA_MAGIC);
    dev_info(&pdev->dev, "BAR0 DMA magic = 0x%x\n", magic);

    d->dma_cpu_addr = dma_alloc_coherent(&pdev->dev, d->dma_size,
                                         &d->dma_handle, GFP_KERNEL);
    if (!d->dma_cpu_addr)
        return -ENOMEM;

    ret = pci_alloc_irq_vectors(pdev, 2, 2, PCI_IRQ_MSIX);
    if (ret < 0) {
        dev_err(&pdev->dev, "pci_alloc_irq_vectors(MSI-X) failed: %d\n", ret);
        goto err_free_dma;
    }

    d->irq_vector = pci_irq_vector(pdev, 0);
    ret = request_irq(d->irq_vector, vfio_bridge_irq_handler, 0, DRV_NAME, d);
    if (ret) {
        dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
        goto err_free_vectors;
    }

    d->irq_eoc = pci_irq_vector(pdev, 1);
    ret = request_irq(d->irq_eoc, vfio_bridge_eoc_handler, 0, DRV_NAME, d);
    if (ret) {
        dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
        goto err_free_vectors_eoc;
    }

    d->miscdev.minor = MISC_DYNAMIC_MINOR;
    d->miscdev.name  = DEV_NAME;
    d->miscdev.fops  = &vfio_bridge_fops;
    d->miscdev.parent = &pdev->dev;

    ret = misc_register(&d->miscdev);
    if (ret) {
        dev_err(&pdev->dev, "misc_register failed: %d\n", ret);
        goto err_free_irq;
    }

    dev_info(&pdev->dev,
             "probed: /dev/%s dma_handle=0x%llx size=%zu irq=%d\n",
             DEV_NAME, (unsigned long long)d->dma_handle,
             d->dma_size, d->irq_vector);

    return 0;

err_free_irq:
    free_irq(d->irq_eoc, d);
err_free_vectors_eoc:
    free_irq(d->irq_vector, d);
err_free_vectors:
    pci_free_irq_vectors(pdev);
err_free_dma:
    dma_free_coherent(&pdev->dev, d->dma_size, d->dma_cpu_addr, d->dma_handle);
    return ret;
}

static void vfio_bridge_remove(struct pci_dev *pdev)
{
    struct vfio_bridge_dev *d = pci_get_drvdata(pdev);

    misc_deregister(&d->miscdev);
    free_irq(d->irq_vector, d);
    free_irq(d->irq_eoc, d);
    pci_free_irq_vectors(pdev);
    dma_free_coherent(&pdev->dev, d->dma_size, d->dma_cpu_addr, d->dma_handle);
}

static struct pci_driver vfio_bridge_pci_driver = {
    .name     = DRV_NAME,
    .id_table = vfio_bridge_pci_ids,
    .probe    = vfio_bridge_probe,
    .remove   = vfio_bridge_remove,
};

module_pci_driver(vfio_bridge_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lorenzo workflow");
MODULE_DESCRIPTION("Host DMA driver for GVSoC vfio-user PCI bridge with private device memory and MSI-X completion");
