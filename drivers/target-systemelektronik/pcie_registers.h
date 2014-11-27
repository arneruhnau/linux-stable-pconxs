#ifndef _PCIE_REGISTERS_H
#define _PCIE_REGISTERS_H

#define PCIE_ATU_VIEWPORT               0x900
#define PCIE_ATU_REGION_INBOUND         (0x1 << 31)
#define PCIE_ATU_REGION_INDEX0          (0x0 << 0)
#define PCIE_ATU_REGION_INDEX1          (0x1 << 0)
#define PCIE_ATU_CR1                    0x904
#define PCIE_ATU_TYPE_MEM               (0x0 << 0)
#define PCIE_ATU_CR2                    0x908
#define PCIE_ATU_ENABLE                 (0x1 << 31)
#define PCIE_ATU_LOWER_BASE             0x90C
#define PCIE_ATU_UPPER_BASE             0x910
#define PCIE_ATU_LIMIT                  0x914
#define PCIE_ATU_LOWER_TARGET           0x918
#define PCIE_ATU_UPPER_TARGET           0x91C

#define TARGET_FPGA_ADC_CONFIG		0
#define TARGET_FPGA_TRIGGER		4
#define TARGET_FPGA_SAMPLES		8
#define TARGET_FPGA_ADC_ONOFF		12

#endif
