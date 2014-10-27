/******************************************************************************
 *
 * CvP code taken from altera_cvp.c -- driver for configuring Altera FPGAs via CvP
 *
 * Written by: Andres Cassinelli <acassine@altera.com>
 *             Altera Corporation
 *
 * Copyright (C) 2012 Altera Corporation. All Rights Reserved.
 *
 * This file is provided under a dual BSD/GPLv2 license.
 *
 *   For the rest:
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

#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/jiffies.h>

#include "pcie_registers.h"
#include "altera_cvp.h"

#define TARGET_FPGA_DRIVER_NAME "target-fpga"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004

struct counted_pages {
	int count;
	struct page *pages;
};

struct fpga_dev {
	struct counted_pages data;
	struct counted_pages counts;
	int interrupts_available;

	u32 unread_data_items;
	u32 counts_position;

	dev_t dev;
	struct cdev cdev;
};

static unsigned short vid = PCI_VENDOR_ID_TARGET;
static unsigned short did = PCI_DEVICE_ID_TARGET_FPGA;
static unsigned int data_pagecount = 16384;
module_param(did, ushort, S_IRUGO);
module_param(vid, ushort, S_IRUGO);
module_param(data_pagecount, int, S_IRUGO);

static DECLARE_COMPLETION(events_available);
static DEFINE_SPINLOCK(sp_unread_data_items);

static struct fpga_dev fpga = {
	.unread_data_items = 0,
	.counts_position = 0,
	.counts = {
		.count = 16
	},
};

static struct altera_cvp_dev cvp_dev; /* contents initialized in altera_cvp_init() */

/* CvP helper functions */

static int altera_cvp_get_offset_and_mask(int bit, int *byte_offset, u8 *mask)
{
	switch (bit) {
		case DATA_ENCRYPTED:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask = MASK_DATA_ENCRYPTED;
			break;
		case DATA_COMPRESSED:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask = MASK_DATA_COMPRESSED;
			break;
		case CVP_CONFIG_READY:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask = MASK_CVP_CONFIG_READY;
			break;
		case CVP_CONFIG_ERROR:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask = MASK_CVP_CONFIG_ERROR;
			break;
		case CVP_EN:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask = MASK_CVP_EN;
			break;
		case USER_MODE:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS;
			*mask =  MASK_USER_MODE;
			break;
		case PLD_CLK_IN_USE:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_STATUS + 1;
			*mask = MASK_PLD_CLK_IN_USE;
			break;
		case CVP_MODE:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_MODE_CTRL;
			*mask = MASK_CVP_MODE;
			break;
		case HIP_CLK_SEL:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_MODE_CTRL;
			*mask = MASK_HIP_CLK_SEL;
			break;
		case CVP_CONFIG:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_PROG_CTRL;
			*mask = MASK_CVP_CONFIG;
			break;
		case START_XFER:
			*byte_offset = OFFSET_VSEC + OFFSET_CVP_PROG_CTRL;
			*mask = MASK_START_XFER;
			break;
		case CVP_CFG_ERR_LATCH:
			*byte_offset = OFFSET_VSEC + OFFSET_UNC_IE_STATUS;
			*mask = MASK_CVP_CFG_ERR_LATCH;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int altera_cvp_read_bit(int bit, u8 *value)
{
	int byte_offset;
	u8 byte_val, byte_mask;
	if (altera_cvp_get_offset_and_mask(bit, &byte_offset, &byte_mask))
		return -EINVAL;
	if (pci_read_config_byte(cvp_dev.pci_dev, byte_offset, &byte_val))
		return -EAGAIN;
	*value = (byte_val & byte_mask) ? 1 : 0;
	return 0;
}

static int altera_cvp_write_bit(int bit, u8 value)
{
	int byte_offset;
	u8 byte_val, byte_mask;

	switch (bit) {
		case CVP_MODE:
		case HIP_CLK_SEL:
		case CVP_CONFIG:
		case START_XFER:
		case CVP_CFG_ERR_LATCH:
			altera_cvp_get_offset_and_mask(bit, &byte_offset, &byte_mask);
			pci_read_config_byte(cvp_dev.pci_dev, byte_offset, &byte_val);
			byte_val = value ? (byte_val | byte_mask) : (byte_val & ~byte_mask);
			pci_write_config_byte(cvp_dev.pci_dev, byte_offset, byte_val);
			return 0;
		default:
			return -EINVAL; /* only the bits above are writeable */
	}
} 

static int altera_cvp_set_num_clks(int num_clks)
{
	if (num_clks < 1 || num_clks > 64)
		return -EINVAL;
	if (num_clks == 64)
		num_clks = 0x00;
	return (pci_write_config_byte(cvp_dev.pci_dev,
					OFFSET_VSEC+OFFSET_CVP_NUMCLKS,
					num_clks));
}

#define NUM_REG_WRITES 244
#define DUMMY_VALUE 0x00000000
/**
 * altera_cvp_switch_clk() - switch between CvP clock and internal clock
 *
 * Issues dummy memory writes to the PCIe HIP, allowing the Control Block to
 * switch between the HIP's CvP clock and the internal clock.
 */
static int altera_cvp_switch_clk(void)
{
	int i;
	altera_cvp_set_num_clks(1);
	for (i = 0; i < NUM_REG_WRITES; i++) {
		iowrite32(DUMMY_VALUE, cvp_dev.wr_addr);
	}
	return 0;
}

static int altera_cvp_set_data_type(void)
{
	int error, num_clks;
	u8 compr, encr;

	if ((error = altera_cvp_read_bit(DATA_COMPRESSED, &compr)) ||
	    (error = altera_cvp_read_bit(DATA_ENCRYPTED, &encr)))
		return error;

	if (compr)
		num_clks = 8;
	else if (encr)
		num_clks = 4;
	else
		num_clks = 1;

	return (altera_cvp_set_num_clks(num_clks));
}

static int altera_cvp_send_data(u32 *data, unsigned long num_words)
{
#ifdef DEBUG
	u8 bit_val;
	unsigned int i;
	for (i = 0; i < num_words; i++) {
		iowrite32(data[i], cvp_dev.wr_addr);
		if ((i + 1) % ERR_CHK_INTERVAL == 0) {
			altera_cvp_read_bit(CVP_CONFIG_ERROR, &bit_val);
			if (bit_val) {
				dev_err(&cvp_dev.pci_dev->dev, "CB detected a CRC error "
								"between words %d and %d\n",
								i + 1 - ERR_CHK_INTERVAL,
								i + 1);
				return -EAGAIN;
			}
		}
	}
	dev_info(&cvp_dev.pci_dev->dev, "A total of %ld 32-bit words were "
					"sent to the FPGA\n", num_words);
#else
	iowrite32_rep(cvp_dev.wr_addr, data, num_words);
#endif /* DEBUG */
	return 0;
}

/* Polls the requested bit until it has the specified value (or until timeout) */
/* Returns 0 once the bit has that value, error code on timeout */
static int altera_cvp_wait_for_bit(int bit, u8 value)
{
	u8 bit_val;
	DECLARE_WAIT_QUEUE_HEAD(cvp_wq);

	altera_cvp_read_bit(bit, &bit_val);
	if (bit_val != value) {
		wait_event_timeout(cvp_wq, 0, MAX_WAIT);
		altera_cvp_read_bit(bit, &bit_val);
		if (bit_val != value) {
			dev_info(&cvp_dev.pci_dev->dev, "Timed out while "
							"polling bit %d\n", bit);
			return -EAGAIN;
		}
	}
	return 0;
}

static int altera_cvp_setup(void)
{
	altera_cvp_write_bit(HIP_CLK_SEL, 1);
	altera_cvp_write_bit(CVP_MODE, 1);
	altera_cvp_switch_clk(); /* allow CB to sense if system reset is issued */
	altera_cvp_write_bit(CVP_CONFIG, 1); /* request CB to begin CvP transfer */

	if (altera_cvp_wait_for_bit(CVP_CONFIG_READY, 1)) /* wait until CB is ready */
		return -EAGAIN;

	altera_cvp_switch_clk();
	altera_cvp_write_bit(START_XFER, 1);
	altera_cvp_set_data_type();
	dev_info(&cvp_dev.pci_dev->dev, "Now starting CvP...\n");
	return 0; /* success */
}

static int altera_cvp_teardown(void)
{
	u8 bit_val;

	/* if necessary, flush remainder buffer */
	if (cvp_dev.remain_size > 0) {
		u32 last_word = 0;
		memcpy(&last_word, cvp_dev.remain, cvp_dev.remain_size);
		altera_cvp_send_data(&last_word, cvp_dev.remain_size);
	}

	altera_cvp_write_bit(START_XFER, 0);
	altera_cvp_write_bit(CVP_CONFIG, 0); /* request CB to end CvP transfer */
	altera_cvp_switch_clk();

	if (altera_cvp_wait_for_bit(CVP_CONFIG_READY, 0)) /* wait until CB is ready */
		return -EAGAIN;

	altera_cvp_read_bit(CVP_CFG_ERR_LATCH, &bit_val);
	if (bit_val) {
		dev_err(&cvp_dev.pci_dev->dev, "Configuration error detected, "
						"CvP has failed\n");
		altera_cvp_write_bit(CVP_CFG_ERR_LATCH, 1); /* clear error bit */
	}

	altera_cvp_write_bit(CVP_MODE, 0);
	altera_cvp_write_bit(HIP_CLK_SEL, 0);

	if (!bit_val) { /* wait for application layer to be ready */
		altera_cvp_wait_for_bit(PLD_CLK_IN_USE, 1);
		altera_cvp_wait_for_bit(USER_MODE, 1);
		dev_info(&cvp_dev.pci_dev->dev, "CvP successful, application "
						"layer now ready\n");
	}
	return 0; /* success */
}

/* Open and close */

int altera_cvp_open(struct inode *inode, struct file *filp)
{
	/* enforce single-open */
	if (!atomic_dec_and_test(&cvp_dev.is_available)) {
		atomic_inc(&cvp_dev.is_available);
		return -EBUSY;
	}

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY) {
		u8 cvp_enabled = 0;
		if (altera_cvp_read_bit(CVP_EN, &cvp_enabled))
			return -EAGAIN;
		if (cvp_enabled) {
			return altera_cvp_setup();
		} else {
			dev_err(&cvp_dev.pci_dev->dev, "CvP is not enabled in "
							"the design on this "
							"FPGA\n");
			return -EOPNOTSUPP;
		}
	}
	return 0; /* success */
}

int altera_cvp_release(struct inode *inode, struct file *filp)
{
	atomic_inc(&cvp_dev.is_available); /* release the device */
	if ((filp->f_flags & O_ACCMODE) != O_RDONLY) {
		return altera_cvp_teardown();
	}
	return 0; /* success */
}

/* Read and write */

ssize_t altera_cvp_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int dev_size = NUM_VSEC_REGS * BYTES_IN_REG;
	int i, byte_offset;
	u8 *out_buf;
	ssize_t ret_val; /* number of bytes successfully read */

	if (*f_pos >= dev_size)
		return 0; /* we're at EOF already */
	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos; /* we can only read until EOF */

	out_buf = kmalloc(count, GFP_KERNEL);

	for (i = 0; i < count; i++) {
		byte_offset = OFFSET_VSEC + *f_pos + i;
		pci_read_config_byte(cvp_dev.pci_dev, byte_offset, &out_buf[i]);
	}

	if (copy_to_user(buf, out_buf, count)) {
		ret_val = -EFAULT;
	} else {
		*f_pos += count;
		ret_val = count;
	}

	kfree(out_buf);
	return ret_val;
}

ssize_t altera_cvp_write(struct file * filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret_val; /* number of bytes successfully transferred */
	u8 *send_buf;
	size_t send_buf_size;

	send_buf_size = count + cvp_dev.remain_size;
	send_buf = kmalloc(send_buf_size, GFP_KERNEL);

	if (cvp_dev.remain_size > 0)
		memcpy(send_buf, cvp_dev.remain, cvp_dev.remain_size);

	if (copy_from_user(send_buf + cvp_dev.remain_size, buf, count)) {
		ret_val = -EFAULT;
		goto exit;
	}

	/* calculate new remainder */
	cvp_dev.remain_size = send_buf_size % 4;

	/* save bytes in new remainder in cvp_dev */
	if (cvp_dev.remain_size > 0)
		memcpy(cvp_dev.remain,
			send_buf + (send_buf_size - cvp_dev.remain_size),
			cvp_dev.remain_size);

	if (altera_cvp_send_data((u32 *)send_buf, send_buf_size / 4)) {
		ret_val = -EAGAIN;
		goto exit;
	}

	*f_pos += count;
	ret_val = count;
exit:
	kfree (send_buf);
	return ret_val;
}

struct file_operations altera_cvp_fops = {
	.owner =   THIS_MODULE,
	.llseek =  no_llseek,
	.read =    altera_cvp_read,
	.write =   altera_cvp_write,
	.open =    altera_cvp_open,
	.release = altera_cvp_release,
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

	while (!pci_is_root_bus(root_complex->bus))
		root_complex = root_complex->bus->self;
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
	if (cp->pages)
		if (!dma_release_from_contiguous(
			&dev->dev, cp->pages, cp->count)
		)
			dev_err(&dev->dev, "Could not release pages\n");
}

static void fpga_release_pages(struct pci_dev *dev)
{
	_fpga_release_pages(dev, &fpga.data);
	_fpga_release_pages(dev, &fpga.counts);
}

static bool _fpga_allocate_pages(struct pci_dev *dev,
				 struct counted_pages *cp)
{
	if (dev_get_cma_area(&dev->dev) == NULL) {
		dev_err(&dev->dev, "CMA area not supported\n");
		return false;
	} else {
		/* Allocate with 1MB (= 2^8 * 4K) alignment */
		cp->pages = dma_alloc_from_contiguous(&dev->dev, cp->count, 8);
		if (cp->pages)
			return true;
		else {
			dev_err(&dev->dev, "Could not alloc %d pages",
				cp->count);
			return false;
		}
	}
}

static bool fpga_allocate_pages(struct pci_dev *dev)
{
	return _fpga_allocate_pages(dev, &fpga.data)
	    && _fpga_allocate_pages(dev, &fpga.counts);
}

static inline int get_recent_count(void)
{
	u32 position;
	struct page *target_page;
	int *buffer;
	int result;
	int n;

	position = fpga.counts_position;
	fpga.counts_position = (fpga.counts_position + 1) & 0xFFFF;
	n = position / (PAGE_SIZE / sizeof(int));
	position = position & 0x3FF;
	target_page = nth_page(fpga.counts.pages, n);
	buffer = (int *)kmap(target_page);
	result = buffer[position];
	kunmap(target_page);
	return result;
}

static irqreturn_t handle_msi_interrupt(int irq, void *data)
{
	int old_value;
	int new_value = get_recent_count();

	spin_lock(&sp_unread_data_items);
	old_value = fpga.unread_data_items;
	fpga.unread_data_items += new_value;
	spin_unlock(&sp_unread_data_items);

	if (!old_value && new_value)
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

		cvp_dev.wr_addr = pcim_iomap(dev, 0, 0);

		cvp_dev.pci_dev = dev; /* store pointer for PCI API calls */

		if (!fpga_allocate_pages(dev))
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
	pci_clear_master(dev);
	pci_disable_pcie_error_reporting(dev);
	fpga_teardown_irq(dev);
	fpga_release_pages(dev);
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

	uaddr = vma->vm_start;
	for (i = 0; i < fpga.data.count; ++i) {
		err = vm_insert_page(vma, uaddr, &fpga.data.pages[i]);
		if (err)
			return err;
		uaddr += PAGE_SIZE;
	}
	return 0;
}

static ssize_t fpga_cdev_read(struct file *filp, char __user *buf,
			      size_t size, loff_t *offset)
{
	int to_read = 0;
	int wait_result = 0;

	wait_result = wait_for_completion_killable_timeout(
		&events_available, msecs_to_jiffies(250)
	);
	spin_lock(&sp_unread_data_items);
	to_read = fpga.unread_data_items;
	fpga.unread_data_items -= to_read;
	spin_unlock(&sp_unread_data_items);

	if (wait_result > 0)
		return copy_to_user(buf, &to_read, sizeof(int));
	else if (wait_result == 0)
		return -ETIME;
	else
		return wait_result;
}

static const struct file_operations fpga_cdev_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= fpga_cdev_open,
	.release	= fpga_cdev_release,
	.read		= fpga_cdev_read,
	.mmap		= fpga_cdev_mmap,
};

static int __init fpga_driver_init(void)
{
	int ret;
	dev_t dev;

	fpga.data.count = data_pagecount;
	ret = alloc_chrdev_region(&dev, 0, 2, TARGET_FPGA_DRIVER_NAME);
	if (ret)
		goto exit;

	fpga.dev = dev;

	ret = pci_register_driver(&fpga_driver);
	if (ret) {
		goto chrdev_exit;
	}

	cdev_init(&fpga.cdev, &fpga_cdev_ops);
	fpga.cdev.owner = THIS_MODULE;
	ret = cdev_add(&fpga.cdev, MKDEV(MAJOR(fpga.dev), 0), 1);
	if (ret < 0)
		goto driver_exit;

	cdev_init(&cvp_dev.cdev, &altera_cvp_fops);
	cvp_dev.cdev.owner = THIS_MODULE;
	ret = cdev_add(&cvp_dev.cdev, MKDEV(MAJOR(fpga.dev), 1), 1);
	if (ret < 0) {
		cdev_del(&fpga.cdev);
		goto driver_exit;
	}

	cvp_dev.remain_size = 0;
	atomic_set(&cvp_dev.is_available, 1);
	return 0;

driver_exit:
	pci_unregister_driver(&fpga_driver);
chrdev_exit:
	unregister_chrdev_region(fpga.dev, 2);
exit:
	return ret;
}

static void __exit fpga_driver_exit(void)
{
	if (atomic_read(&cvp_dev.is_available))
	{
		cdev_del(&fpga.cdev);
		cdev_del(&cvp_dev.cdev);
		pci_unregister_driver(&fpga_driver);
		unregister_chrdev_region(fpga.dev, 2);
	}
}

module_init(fpga_driver_init);
module_exit(fpga_driver_exit);

MODULE_AUTHOR("Target Systemelektronik");
MODULE_DESCRIPTION("PCI Express driver module for our FPGA");
MODULE_LICENSE("GPL");
