/******************************************************************************
 *
 *   Copyright (C) 2014  Target Systemelektronik GmbH & Co. KG.
 *   All rights reserved.
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *****************************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/aer.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <uapi/linux/pci_regs.h>
#include <uapi/linux/if.h>
#include <linux/highmem.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/jiffies.h>

#include "pcie_registers.h"

#define TARGET_FPGA_DRIVER_NAME "target-fpga"
#define TARGET_FPGA_CLASS_NAME "target-fpga"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004

struct fpga_ringbuffer {
	int size;
	void *start;
	dma_addr_t dma_handle;
};

struct fpga_dev {
	struct fpga_ringbuffer data;
	struct fpga_ringbuffer counts;
	int interrupts_available;

	atomic_t unread_data_items;
	u32 counts_position;

	unsigned int major_device_number;
	dev_t dev;
	struct cdev cdev;
	void __iomem *bar1;
};

static struct class *device_class;

static unsigned short vid = PCI_VENDOR_ID_TARGET;
static unsigned short did = PCI_DEVICE_ID_TARGET_FPGA;
static unsigned int data_size = 16384 * PAGE_SIZE;
module_param(did, ushort, S_IRUGO);
module_param(vid, ushort, S_IRUGO);
module_param(data_size, int, S_IRUGO);

static DECLARE_COMPLETION(events_available);

static struct fpga_dev fpga = {
	.unread_data_items = ATOMIC_INIT(0),
	.counts_position = 0,
	.counts = {
		.size = 16 * PAGE_SIZE
	},
};

static void _dw_pcie_prog_viewport_inbound(
	struct pci_dev *dev, u32 viewport,
	u64 fpga_base, u64 ram_base, u64 size)
{
	pci_write_config_dword(dev, PCIE_ATU_VIEWPORT,
			       PCIE_ATU_REGION_INBOUND | viewport);
	pci_write_config_dword(dev, PCIE_ATU_LOWER_BASE,
			       fpga_base);
	pci_write_config_dword(dev, PCIE_ATU_UPPER_BASE,
			       fpga_base >> 32);
	pci_write_config_dword(dev, PCIE_ATU_LIMIT,
			       fpga_base + size - 1);
	pci_write_config_dword(dev, PCIE_ATU_LOWER_TARGET,
			       ram_base);
	pci_write_config_dword(dev, PCIE_ATU_UPPER_TARGET,
			       ram_base + size - 1);
	pci_write_config_dword(dev, PCIE_ATU_CR1,
			       PCIE_ATU_TYPE_MEM);
	pci_write_config_dword(dev, PCIE_ATU_CR2,
			       PCIE_ATU_ENABLE);

	dev_info(&dev->dev,
		"Viewpoint:\t0x%04X\n"
		"Size:\t\t0x%016llX\n"
		"FPGA-Start:\t0x%016llX\n"
		"FPGA-End:\t0x%016llX\n"
		"Ram-Start:\t0x%016llX\n"
		"Ram-End:\t0x%016llX\n",
		viewport,
		size,
		fpga_base,
		fpga_base + size - 1,
		ram_base,
		ram_base + size - 1
	);
}

static void dw_pcie_prog_viewports_inbound(struct pci_dev *dev)
{
	struct pci_dev *root_complex = dev;
const int offset = 0x40000000;
	while (!pci_is_root_bus(root_complex->bus))
		root_complex = root_complex->bus->self;
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX0,
		offset + 0,
		(u64)fpga.data.dma_handle,
		fpga.data.size
	);
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX1,
		offset + fpga.data.size,
		(u64)fpga.counts.dma_handle,
		fpga.counts.size
	);
}

static bool _fpga_allocate_buffer(struct pci_dev *dev,
				    struct fpga_ringbuffer *pws)
{
	pws->start = dmam_alloc_coherent(&dev->dev, pws->size, &pws->dma_handle, GFP_USER | __GFP_ZERO);
	if (pws->start == NULL)
	{
		dev_err(&dev->dev, "Could not alloc %d bytes", pws->size);
		return false;
	}
	return true;
}

static bool fpga_allocate_buffers(struct pci_dev *dev)
{
	if(!_fpga_allocate_buffer(dev, &fpga.data))
		return false;
	return _fpga_allocate_buffer(dev, &fpga.counts);
}

static irqreturn_t handle_msi_interrupt(int irq, void *data)
{
	int to_add;
	u32 position;

	position = fpga.counts_position;
	fpga.counts_position = (position + 1) & 16383;
	to_add = ((int*)fpga.counts.start)[position];
	if (to_add != 0x4000)
		printk(KERN_ERR "Read %u at %u\n", to_add, position);
	if (atomic_add_return(to_add, &fpga.unread_data_items) == to_add)
		complete(&events_available);
	return IRQ_HANDLED;
}

static int fpga_setup_irq(struct pci_dev *dev)
{
	int irq;
	int end;
	int ret;
	char *name = "fpga-msi";

	fpga.interrupts_available = pci_enable_msi_range(dev, 1, 4);
	if (fpga.interrupts_available < 0) {
		dev_err(&dev->dev, "Could not request msi range [1,4]\n");
		return fpga.interrupts_available;
	}
	dev_info(&dev->dev, "Enabled %d interrupts\n",
		 fpga.interrupts_available);
	end = dev->irq + fpga.interrupts_available - 1;
	for (irq = dev->irq; irq <= end; ++irq) {
		snprintf(name, IFNAMSIZ, "fpga-msi-%d", irq);
		name[IFNAMSIZ-1] = 0;
		ret = devm_request_irq(&dev->dev, irq, handle_msi_interrupt,
				       0, name, dev);
		if (ret) {
			dev_err(&dev->dev, "Failed to request irq %d\n", irq);
			return ret;
		}
	}
	return 0;
}

static void fpga_teardown_irq(struct pci_dev *dev)
{
	int irq;
	int end = dev->irq + fpga.interrupts_available - 1;

	for (irq = dev->irq; irq <= end; ++irq)
		devm_free_irq(&dev->dev, irq, dev);
	pci_disable_msi(dev);
}

static int fpga_driver_probe(struct pci_dev *dev,
			     const struct pci_device_id *id)
{
	int ret;

	if ((dev->vendor == vid) && (dev->device == did)) {
		ret = pcim_enable_device(dev);
		if (ret) {
			dev_err(&dev->dev, "pci_enable_device() failed\n");
			return ret;
		}
		ret = pcim_iomap_regions(dev, 0, TARGET_FPGA_DRIVER_NAME);
		if (ret) {
			dev_err(&dev->dev, "pcim_iomap_regions() failed\n");
			return ret;
		}
		fpga.bar1 = pcim_iomap_table(dev)[1];
		if (!fpga_allocate_buffers(dev))
			return -ENOMEM;
		dw_pcie_prog_viewports_inbound(dev);
		ret = fpga_setup_irq(dev);
		if (ret)
			return ret;
		if (pci_find_ext_capability(dev, PCI_EXT_CAP_ID_ERR))
			ret = pci_enable_pcie_error_reporting(dev);
		else
			dev_info(&dev->dev, "AER not supported\n");

		pci_set_master(dev);
		return ret;
	} else
		return -ENODEV;
}

static void fpga_driver_remove(struct pci_dev *dev)
{
	device_destroy(device_class, MKDEV(MAJOR(fpga.dev), 0));
	class_destroy(device_class);
	pci_clear_master(dev);
	pci_disable_pcie_error_reporting(dev);
	fpga_teardown_irq(dev);
}


/* we check for the configured vid/did dynamically for now */
static const struct pci_device_id fpga_driver_tbl[] = {
	{ PCI_DEVICE(PCI_ANY_ID, PCI_ANY_ID) },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, fpga_driver_tbl);

static struct pci_driver fpga_driver = {
	.name		= TARGET_FPGA_DRIVER_NAME,
	.id_table = fpga_driver_tbl,
	.probe		= fpga_driver_probe,
	.remove		= fpga_driver_remove,
};

static int fpga_cdev_open(struct inode *inode, struct file *filp)
{
	return nonseekable_open(inode, filp);
}

static int fpga_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int fpga_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long uaddr;
	int i, err;

	struct page *start = virt_to_page(fpga.data.start);
	uaddr = vma->vm_start;
	for (i = 0; i < fpga.data.size / PAGE_SIZE; ++i) {
		err = vm_insert_page(vma, uaddr, nth_page(start, i));
		if (err)
			return err;
		uaddr += PAGE_SIZE;
	}
	return 0;
}

static ssize_t fpga_cdev_read(struct file *filp, char __user *buf,
			      size_t size, loff_t *offset)
{
	int to_read, wait_result;

	wait_result = wait_for_completion_killable_timeout(
		&events_available, msecs_to_jiffies(250)
	);

	if (wait_result > 0) {
		to_read = atomic_xchg(&fpga.unread_data_items, 0);
		return copy_to_user(buf, &to_read, sizeof(int));
	} else if (wait_result == 0)
		return -ETIME;
	return wait_result;
}

static void bar_write16(u16 value, int offset)
{
	printk(KERN_ERR "Write %u to bar1@%d", value, offset);
	iowrite16(value, fpga.bar1 + offset);
}

static ssize_t fpga_cdev_write(struct file *filp, const char __user *buf,
			       size_t size, loff_t *offset)
{
	/*

		buf			length
		on			3
		off			4
		samples \d{1,4}		9-13
		trigger \d{1,4}		9-13
	*/
#define MAXIMAL_INPUT_LENGTH 13

	long input_length;
	char *copy;
	int uservalue;

	input_length = strnlen_user(buf, MAXIMAL_INPUT_LENGTH);
	if (input_length < 0)
		return input_length;
	if (input_length == 0)
		return -EFAULT;
	if (input_length > MAXIMAL_INPUT_LENGTH)
		return -EINVAL;

	copy = kzalloc(input_length, GFP_KERNEL);
	if (copy == NULL)
		return -ENOMEM;
	strncpy_from_user(copy, buf, input_length);

	if (strcmp(copy, "on") == 0)
	{
		bar_write16(1, TARGET_FPGA_ADC_ONOFF);
	}
	else if (strcmp(copy, "off") == 0)
	{
		bar_write16(0, TARGET_FPGA_ADC_ONOFF);
	}
	else if (sscanf(copy, "samples %i", &uservalue) == 1)
	{
		bar_write16((u16)uservalue, TARGET_FPGA_SAMPLES);
	}
	else if (sscanf(copy, "trigger %i", &uservalue) == 1)
	{
		bar_write16((u16)uservalue, TARGET_FPGA_TRIGGER);
	}

	kfree(copy);
	return -EINVAL;
}

static const struct file_operations fpga_cdev_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= fpga_cdev_open,
	.release	= fpga_cdev_release,
	.read		= fpga_cdev_read,
	.write		= fpga_cdev_write,
	.mmap		= fpga_cdev_mmap,
};

static int __init fpga_driver_init(void)
{
	int ret;
	dev_t dev;
	struct device *device;

	fpga.data.size = data_size;
	ret = alloc_chrdev_region(&dev, 0, 1, TARGET_FPGA_DRIVER_NAME);
	if (ret)
		goto exit;

	fpga.dev = dev;
	fpga.major_device_number = MAJOR(dev);

	ret = pci_register_driver(&fpga_driver);
	if (ret) {
		unregister_chrdev_region(MKDEV(fpga.major_device_number, 0), 1);
		goto exit;
	}

	cdev_init(&fpga.cdev, &fpga_cdev_ops);
	fpga.cdev.owner = THIS_MODULE;
	ret = cdev_add(&fpga.cdev, fpga.dev, 1);
	if (ret) {
		pci_unregister_driver(&fpga_driver);
		unregister_chrdev_region(MKDEV(fpga.major_device_number, 0), 1);
		goto exit;
	}

	device_class = class_create(THIS_MODULE, TARGET_FPGA_CLASS_NAME);
	if (IS_ERR(device_class)) {
		ret = PTR_ERR(device_class);
		goto driver_exit;
	}

	device = device_create(device_class, NULL, MKDEV(MAJOR(fpga.dev), 0), NULL, "target-fpga");
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		printk(KERN_WARNING "Error %d while trying to create target-fpga", ret);
		goto driver_exit;
	}
	return 0;

driver_exit:
	pci_unregister_driver(&fpga_driver);
	unregister_chrdev_region(fpga.dev, 2);
exit:
	return ret;
}

static void __exit fpga_driver_exit(void)
{
	cdev_del(&fpga.cdev);
	unregister_chrdev_region(MKDEV(fpga.major_device_number, 0), 1);
	pci_unregister_driver(&fpga_driver);
}

module_init(fpga_driver_init);
module_exit(fpga_driver_exit);

MODULE_AUTHOR("Target Systemelektronik");
MODULE_DESCRIPTION("PCI Express driver module for our FPGA");
MODULE_LICENSE("GPL");
