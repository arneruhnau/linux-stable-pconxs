/******************************************************************************
 *
 *   Copyright (C) 2014  Target Systemelektronik GmbH & Co. KG. All rights reserved.
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

#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/fs.h>

#include <linux/spinlock.h>
#include <linux/atomic.h>

#include "pcie_registers.h"

#define TARGET_FPGA_DRIVER_NAME "Target FPGA"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004

struct counted_pages {
	int count;
	struct page *pages;
	ssize_t item_size;
};

struct fpga_dev {
	struct counted_pages data;
	struct counted_pages counts;
	int interrupts_available;

	atomic_t unread_data_items;
};


static unsigned short vid = PCI_VENDOR_ID_TARGET;
static unsigned short did = PCI_DEVICE_ID_TARGET_FPGA;
static unsigned int data_pagecount = 1;
static unsigned int data_itemsize = sizeof(u32);
static unsigned int count_pagecount = 1;
static unsigned int count_itemsize = sizeof(u32);
module_param(did, ushort, S_IRUGO);
module_param(vid, ushort, S_IRUGO);
module_param(data_pagecount, int, S_IRUGO);
module_param(data_itemsize, int, S_IRUGO);
module_param(count_pagecount, int, S_IRUGO);
module_param(count_itemsize, int, S_IRUGO);

static struct fpga_dev fpga = {
	.unread_data_items = ATOMIC_INIT(0)
};

static ssize_t maxitems_show(struct device *dev, struct attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", PAGE_SIZE * fpga.data.count / fpga.data.item_size);
}

static ssize_t unread_items_show(struct device *dev, struct attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", atomic_read(&fpga.unread_data_items));
}

static ssize_t unread_items_store(struct device *dev, struct attribute *attr, const char *buf, size_t count)
{
	int read_items;
	if(kstrtoint(buf, 0, &read_items))
		return -EINVAL;
	atomic_sub(read_items, &fpga.unread_data_items);
	return count;
}

static DEVICE_ATTR_RO(maxitems);
static DEVICE_ATTR_RW(unread_items);

static void _dw_pcie_prog_viewport_inbound(struct pci_dev *dev, u32 viewport, u64 fpga_base, u64 ram_base, u64 size)
{
        pci_write_config_dword(dev, PCIE_ATU_VIEWPORT,		PCIE_ATU_REGION_INBOUND |
								viewport);
        pci_write_config_dword(dev, PCIE_ATU_LOWER_BASE, 	fpga_base);
        pci_write_config_dword(dev, PCIE_ATU_UPPER_BASE, 	fpga_base >> 32);
        pci_write_config_dword(dev, PCIE_ATU_LIMIT, 		fpga_base + size - 1);
        pci_write_config_dword(dev, PCIE_ATU_LOWER_TARGET, 	ram_base);
        pci_write_config_dword(dev, PCIE_ATU_UPPER_TARGET, 	ram_base + size - 1);
        pci_write_config_dword(dev, PCIE_ATU_CR1, 		PCIE_ATU_TYPE_MEM);
        pci_write_config_dword(dev, PCIE_ATU_CR2, 		PCIE_ATU_ENABLE);

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

	while(!pci_is_root_bus(root_complex->bus)) {
		root_complex = root_complex->bus->self;
	}
	_dw_pcie_prog_viewport_inbound(
		root_complex, 
		PCIE_ATU_REGION_INDEX0,
		0, 
		page_to_phys(fpga.data.pages),
		PAGE_SIZE * fpga.data.count
	);
	_dw_pcie_prog_viewport_inbound(
		root_complex, 
		PCIE_ATU_REGION_INDEX1,
		PAGE_SIZE * fpga.data.count,
		page_to_phys(fpga.counts.pages),
		PAGE_SIZE * fpga.counts.count
	);
}

static void _fpga_release_pages(struct pci_dev *dev, struct counted_pages *cp)
{
	if(cp->pages) {
		if(!dma_release_from_contiguous(&dev->dev, cp->pages, cp->count))
			dev_err(&dev->dev, "Could not release pages\n");
		else
			dev_info(&dev->dev, "Released %d pages from contiguous\n", cp->count);
	}
}

static void fpga_release_pages(struct pci_dev *dev) {
	_fpga_release_pages(dev, &fpga.data);
	_fpga_release_pages(dev, &fpga.counts);
}

static bool _fpga_allocate_pages(struct pci_dev *dev, struct counted_pages *cp) {
	if(dev_get_cma_area(&dev->dev) == NULL)
	{
		dev_err(&dev->dev, "CMA area not supported\n");
		return false;
	}
	else {
		cp->pages = dma_alloc_from_contiguous(&dev->dev, cp->count, 8); // 8 = 2^8 == 256-page-alignment
		if(cp->pages)
			return true;
		else {
			dev_err(&dev->dev, "Could not alloc %d cma pages", cp->count);
			return false;
		}
	}
}

static bool fpga_allocate_pages(struct pci_dev *dev) {
	bool ret = _fpga_allocate_pages(dev, &fpga.data);
	if(!ret)
		return ret;
	ret = _fpga_allocate_pages(dev, &fpga.counts);
	return ret;
}

static irqreturn_t handle_msi_interrupt(int irq, void *data) {
	/* read from INBOUND1 the most recent count */
	atomic_add(2, &fpga.unread_data_items);
	return IRQ_HANDLED;
}

static int fpga_setup_irq(struct pci_dev *dev) {
	int irq;
	int end;
	int ret;
	char *name = "fpga-msi";

	fpga.interrupts_available = pci_enable_msi_range(dev, 1, 4);
	if(fpga.interrupts_available < 0)
	{
		dev_err(&dev->dev, "Could not request msi range [1,4]\n");
		return fpga.interrupts_available;
	}
	dev_info(&dev->dev, "Enabled %d interrupts\n", fpga.interrupts_available);
	end = dev->irq + fpga.interrupts_available - 1;
	for(irq = dev->irq; irq <= end; ++irq)
	{
		snprintf(name, IFNAMSIZ, "fpga-msi-%d", irq);
		name[IFNAMSIZ-1] = 0;
		ret = devm_request_irq(&dev->dev, irq, handle_msi_interrupt, 0, name, dev);
		if(ret) {
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
	for(irq = dev->irq; irq <= end; ++irq)
		devm_free_irq(&dev->dev, irq, dev);
	pci_disable_msi(dev);
}

static int fpga_driver_probe(struct pci_dev *dev, const struct pci_device_id *id) {
	int ret = 0;
	if ((dev->vendor == vid) && (dev->device == did)) 
	{
		ret = pcim_enable_device(dev);
		if (ret) {
			dev_err(&dev->dev, "pci_enable_device() failed\n");
			return ret;
		}
		dev_info(&dev->dev, "Found and enabled PCI device with "
					"VID 0x%04X, DID 0x%04X\n", vid, did);
		ret = pcim_iomap_regions(dev, 0, TARGET_FPGA_DRIVER_NAME);
		if(ret)
		{
			dev_err(&dev->dev, "pcim_iomap_regions() failed\n");
			return ret;
		}
		if(!fpga_allocate_pages(dev))	
			return -ENOMEM;
		dw_pcie_prog_viewports_inbound(dev);
		ret = fpga_setup_irq(dev);
		if(ret)
			return ret;
		if(pci_find_ext_capability(dev, PCI_EXT_CAP_ID_ERR))
			ret = pci_enable_pcie_error_reporting(dev);
		else
			dev_info(&dev->dev, "AER not supported\n");

		pci_set_master(dev);
		device_create_file(&dev->dev, &dev_attr_maxitems);
		device_create_file(&dev->dev, &dev_attr_unread_items);
		return ret;
	}
	else {
		dev_err(&dev->dev, "This PCI device does not match "
					"VID 0x%04X, DID 0x%04X\n", vid, did);
		return -ENODEV;
	}
}

static void fpga_driver_remove(struct pci_dev *dev) {
	device_remove_file(&dev->dev, &dev_attr_maxitems);
	device_remove_file(&dev->dev, &dev_attr_unread_items);
	pci_clear_master(dev);
	pci_disable_pcie_error_reporting(dev);
	fpga_teardown_irq(dev);
	fpga_release_pages(dev);
	return;
}


// we check for the configured vid/did dynamically for now
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

static int __init fpga_driver_init(void)
{
	fpga.data.count = data_pagecount;
	fpga.data.item_size = data_itemsize;
	fpga.counts.count = count_pagecount;
	fpga.counts.item_size = count_itemsize;
	return pci_register_driver(&fpga_driver);
}

static void __exit fpga_driver_exit(void)
{
	pci_unregister_driver(&fpga_driver);
}

module_init(fpga_driver_init);
module_exit(fpga_driver_exit);

MODULE_AUTHOR("Target Systemelektronik");
MODULE_DESCRIPTION("PCI Express driver module for our FPGA");
MODULE_LICENSE("GPL");
