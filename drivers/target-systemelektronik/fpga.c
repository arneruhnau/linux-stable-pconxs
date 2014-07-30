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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <uapi/linux/pci_regs.h>

#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/fs.h>

#include "pcie_registers.h"

#define TARGET_FPGA_DRIVER_NAME "Target FPGA"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004

static unsigned short vid = PCI_VENDOR_ID_TARGET;
static unsigned short did = PCI_DEVICE_ID_TARGET_FPGA;
static unsigned int pagecount = 1;
module_param(did, ushort, S_IRUGO);
module_param(vid, ushort, S_IRUGO);
module_param(pagecount, int, S_IRUGO);

static void dw_pcie_prog_viewport_inbound0(struct pci_dev *dev, u64 fpga_base, u64 ram_base, u64 size)
{
        /* Program viewport 0 : INBOUND : MEM */
	
        pci_write_config_dword(dev, PCIE_ATU_VIEWPORT,		PCIE_ATU_REGION_INBOUND |
								PCIE_ATU_REGION_INDEX0);
        pci_write_config_dword(dev, PCIE_ATU_LOWER_BASE, 	fpga_base);
        pci_write_config_dword(dev, PCIE_ATU_UPPER_BASE, 	fpga_base >> 32);
        pci_write_config_dword(dev, PCIE_ATU_LIMIT, 		fpga_base + size - 1);
        pci_write_config_dword(dev, PCIE_ATU_LOWER_TARGET, 	ram_base);
        pci_write_config_dword(dev, PCIE_ATU_UPPER_TARGET, 	ram_base + size - 1);
        pci_write_config_dword(dev, PCIE_ATU_CR1, 		PCIE_ATU_TYPE_MEM);
        pci_write_config_dword(dev, PCIE_ATU_CR2, 		PCIE_ATU_ENABLE);

	dev_info(&dev->dev, 
		"Viewpoint:\n"
		"Size:\t\t0x%016llX\n"
		"FPGA-Start:\t0x%016llX\n"
		"FPGA-End:\t0x%016llX\n" 
		"Ram-Start:\t0x%016llX\n" 
		"Ram-End:\t0x%016llX\n",
		size,
		fpga_base,
		fpga_base + size - 1,
		ram_base,
		ram_base + size - 1
	);
}

static struct page *cma_pages;

static void fpga_release_cma_pages(struct pci_dev *dev) {
	if(cma_pages) {
		if(!dma_release_from_contiguous(&dev->dev, cma_pages, pagecount))
			dev_err(&dev->dev, "Could not release cma pages\n");
		else
			dev_info(&dev->dev, "Released %d pages from contiguous\n", pagecount);
	}
}

static bool fpga_allocate_cma_pages(struct pci_dev *dev) {
	if(dev_get_cma_area(&dev->dev) == NULL)
	{
		dev_err(&dev->dev, "CMA area not supported\n");
		return false;
	}
	else {
		cma_pages = dma_alloc_from_contiguous(&dev->dev, pagecount, 8); // 8 = 2^8 == 256-page-alignment
		if(cma_pages) {
			dev_info(&dev->dev, "Allocated %d pages from contiguous\n", pagecount);
			return true;
		}
		else {
			dev_err(&dev->dev, "Could not alloc cma pages");
			return false;
		}
	}
}

static int fpga_driver_probe(struct pci_dev *dev, const struct pci_device_id *id) {
	int ret = 0;
	struct pci_dev *root_complex = dev;

	while(!pci_is_root_bus(root_complex->bus)) {
		root_complex = root_complex->bus->self;
	}
	if ((dev->vendor == vid) && (dev->device == did)) 
	{
		ret = pci_enable_device(dev);
		if (ret) {
			dev_err(&dev->dev, "pci_enable_device() failed\n");
			return ret;
		}
		dev_info(&dev->dev, "Found and enabled PCI device with "
					"VID 0x%04X, DID 0x%04X\n", vid, did);
		ret = pci_request_regions(dev, TARGET_FPGA_DRIVER_NAME);
		if(ret)
		{
			dev_err(&dev->dev, "pci_request_regions() failed\n");
			return ret;
		}
		pci_set_master(dev);
		if(fpga_allocate_cma_pages(dev))	
			dw_pcie_prog_viewport_inbound0(root_complex, 0x00000000, page_to_phys(cma_pages), 
						PAGE_SIZE * pagecount);
		return 0;
	}
	else {
		dev_err(&dev->dev, "This PCI device does not match "
					"VID 0x%04X, DID 0x%04X\n", vid, did);
		return -ENODEV;
	}
}

static void fpga_driver_remove(struct pci_dev *dev) {
	pci_clear_master(dev);
	fpga_release_cma_pages(dev);
        pci_disable_device(dev);
        pci_release_regions(dev);
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
	int ret = 0;
	ret = pci_register_driver(&fpga_driver);
	if(!ret) {
	}
	return ret;
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
