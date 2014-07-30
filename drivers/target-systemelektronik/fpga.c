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

#define TARGET_FPGA_DRIVER_NAME "Target FPGA"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004

static unsigned short vid = PCI_VENDOR_ID_TARGET;
static unsigned short did = PCI_DEVICE_ID_TARGET_FPGA;
static unsigned int pagecount = 1;
module_param(did, ushort, S_IRUGO);
module_param(vid, ushort, S_IRUGO);
module_param(pagecount, int, S_IRUGO);
 

static int fpga_driver_probe(struct pci_dev *dev, const struct pci_device_id *id) {
	u32 config_value = 0x0;
	int ret = 0;
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
		ret = pci_read_config_dword(dev, 0x018, &config_value);
		if (ret)
			dev_err(&dev->dev, "pci_read_config_dword failed: 0x%04X\n", ret);
		else
			dev_info(&dev->dev, "0x018: 0x%04X\n", config_value);

		if(dev_get_cma_area(&dev->dev) == NULL)
		{
			dev_err(&dev->dev, "CMA area not supported\n");
		}
		else {

			struct page* cma_pages = dma_alloc_from_contiguous(&dev->dev, pagecount, 8); // 8 = 2^8 == 256-page-alignment
			if(cma_pages) {
				dev_info(&dev->dev, "Allocated %d pages from contiguous\n", pagecount);
				if(!dma_release_from_contiguous(&dev->dev, cma_pages, pagecount))
					dev_err(&dev->dev, "Could not release cma pages\n");
				else
					dev_info(&dev->dev, "Released %d pages from contiguous\n", pagecount);
			}
			else {
				dev_err(&dev->dev, "Could not alloc cma pages");
			}
}
		
		return 0;
	}
	else {
		dev_err(&dev->dev, "This PCI device does not match "
					"VID 0x%04X, DID 0x%04X\n", vid, did);
		return -ENODEV;
	}
}

static void fpga_driver_remove(struct pci_dev *dev) {
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
