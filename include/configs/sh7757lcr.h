/*
 * Configuation settings for the sh7757lcr board
 *
 * Copyright (C) 2011 Renesas Solutions Corp.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SH7757LCR_H
#define __SH7757LCR_H

#define CONFIG_CPU_SH7757	1
#define CONFIG_SH7757LCR	1
#define CONFIG_SH7757LCR_DDR_ECC	1

#define CONFIG_SYS_TEXT_BASE	0x8ef80000

#define CONFIG_DISPLAY_BOARDINFO
#undef	CONFIG_SHOW_BOOT_PROGRESS

/* MEMORY */
#define SH7757LCR_SDRAM_BASE		(0x80000000)
#define SH7757LCR_SDRAM_SIZE		(240 * 1024 * 1024)
#define SH7757LCR_SDRAM_ECC_SETTING	0x0f000000	/* 240MByte */
#define SH7757LCR_SDRAM_DVC_SIZE	(16 * 1024 * 1024)

#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PBSIZE		256
#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200 }

/* SCIF */
#define CONFIG_CONS_SCIF2	1

#define CONFIG_SYS_MEMTEST_START	(SH7757LCR_SDRAM_BASE)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + \
					 224 * 1024 * 1024)
#undef	CONFIG_SYS_ALT_MEMTEST
#undef	CONFIG_SYS_MEMTEST_SCRATCH
#undef	CONFIG_SYS_LOADS_BAUD_CHANGE

#define CONFIG_SYS_SDRAM_BASE		(SH7757LCR_SDRAM_BASE)
#define CONFIG_SYS_SDRAM_SIZE		(SH7757LCR_SDRAM_SIZE)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + \
					 (128 + 16) * 1024 * 1024)

#define CONFIG_SYS_MONITOR_BASE		0x00000000
#define CONFIG_SYS_MONITOR_LEN		(256 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(4 * 1024 * 1024)
#define CONFIG_SYS_BOOTMAPSZ		(8 * 1024 * 1024)

/* Ether */
#define CONFIG_SH_ETHER_USE_PORT	0
#define CONFIG_SH_ETHER_PHY_ADDR	1
#define CONFIG_SH_ETHER_CACHE_WRITEBACK	1
#define CONFIG_BITBANGMII
#define CONFIG_BITBANGMII_MULTI
#define CONFIG_SH_ETHER_PHY_MODE PHY_INTERFACE_MODE_MII

#define SH7757LCR_ETHERNET_MAC_BASE_SPI	0x000b0000
#define SH7757LCR_SPI_SECTOR_SIZE	(64 * 1024)
#define SH7757LCR_ETHERNET_MAC_BASE	SH7757LCR_ETHERNET_MAC_BASE_SPI
#define SH7757LCR_ETHERNET_MAC_SIZE	17
#define SH7757LCR_ETHERNET_NUM_CH	2

/* Gigabit Ether */
#define SH7757LCR_GIGA_ETHERNET_NUM_CH	2

/* SPI */
#define CONFIG_SH_SPI			1
#define CONFIG_SH_SPI_BASE		0xfe002000

/* MMCIF */
#define CONFIG_SH_MMCIF			1
#define CONFIG_SH_MMCIF_ADDR		0xffcb0000
#define CONFIG_SH_MMCIF_CLK		48000000

/* SH7757 board */
#define SH7757LCR_SDRAM_PHYS_TOP	0x40000000
#define SH7757LCR_GRA_OFFSET		0x1f000000
#define SH7757LCR_PCIEBRG_ADDR_B0	0x000a0000
#define SH7757LCR_PCIEBRG_SIZE_B0	(64 * 1024)
#define SH7757LCR_PCIEBRG_ADDR		0x00090000
#define SH7757LCR_PCIEBRG_SIZE		(96 * 1024)

/* ENV setting */
#define CONFIG_ENV_IS_EMBEDDED
#define CONFIG_ENV_SECT_SIZE	(64 * 1024)
#define CONFIG_ENV_ADDR		(0x00080000)
#define CONFIG_ENV_OFFSET	(CONFIG_ENV_ADDR)
#define CONFIG_ENV_OVERWRITE	1
#define CONFIG_ENV_SIZE		(CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND	(CONFIG_ENV_SECT_SIZE)
#define CONFIG_EXTRA_ENV_SETTINGS				\
		"netboot=bootp; bootm\0"

/* Board Clock */
#define CONFIG_SYS_CLK_FREQ	48000000
#define CONFIG_SH_TMU_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SYS_TMU_CLK_DIV	4
#endif	/* __SH7757LCR_H */
