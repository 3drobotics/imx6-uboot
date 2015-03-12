/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 */

#ifndef __MX6QSABRESD_CONFIG_H
#define __MX6QSABRESD_CONFIG_H

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
//#ifndef CONFIG_CONSOLE_DEV
//#define CONFIG_CONSOLE_DEV		"ttymxc0"
//#endif
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"

#include "mx6qsabre_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_GPIO
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
//#define CONFIG_USB_HOST_ETHER
//#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1 /* Enabled USB controller number */

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC3 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user partition */

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   (0|(IMX_GPIO_NR(4, 9)<<8))
#endif

//#define CONFIG_CMD_HDMIDETECT
//#define CONFIG_SPLASH_SCREEN
//#define CONFIG_MXC_EPDC

/*LDO BYPASS removal*/
#undef CONFIG_LDO_BYPASS_CHECK

/*Ethernet removal*/
#undef CONFIG_CMD_PING
#undef CONFIG_CMD_DHCP
#undef CONFIG_CMD_MII
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_FEC_MXC
#undef CONFIG_MII
#undef IMX_FEC_BASE 
#undef CONFIG_FEC_XCV_TYPE
#undef CONFIG_ETHPRIME
#undef CONFIG_FEC_MXC_PHYADDR
#undef CONFIG_PHYLIB
#undef CONFIG_PHY_ATHEROS

//Redo the environment variables
#undef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS \
        "initrd_addr=0x12C00000\0" \
        "initrd_high=0xffffffff\0" \
        CONFIG_MFG_ENV_SETTINGS \
        "uimage=uImage\0" \
        "fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
        "fdt_addr=0x18000000\0" \
        "boot_fdt=try\0" \
        "ip_dyn=yes\0" \
        "console=" CONFIG_CONSOLE_DEV "\0" \
        "fdt_high=0xffffffff\0"   \
        "initrd_high=0xffffffff\0" \
        "bootdelay=0\0" \
        "factoryReset=0\0" \
        CONFIG_MMC_DEV_SET \
        "\0" \
        "mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
        "mmcroot=" CONFIG_MMCROOT " rootwait ro\0" \
        "smp=" CONFIG_SYS_NOSMP "\0"\
        "mmcargs=setenv bootargs console=${console},${baudrate} ${smp} " \
                "root=${mmcroot} video=mxcfb0:dev=hdmi,1280x720M@60 consoleblank=0 factoryreset=${factoryReset} vt.global_cursor_default=0 \0" \
        "loadbootscript=" \
                "fatload mmc ${mmcdev}:${mmcpart} ${loadaddr};\0" \
        "bootscript=echo Running bootscript from mmc ...; " \
                "source\0" \
        "loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
        "loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "mmcboot=echo Booting from mmc ...; " \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run loadfdt; then " \
                                "bootm ${loadaddr} - ${fdt_addr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootm; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootm; " \
                "fi;\0" \

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
        "mmc dev ${mmcdev};" \
        "if mmc rescan; then " \
                "if run loadbootscript; then " \
                "run bootscript; " \
                "else " \
                    "setenv mmcpart 2; "\
                    "setenv mmcroot /dev/mmcblk0p2 rootwait ro; "\
                        "if run loaduimage; then "\
                             "run mmcboot; " \
                         "fi; " \
                         "setenv mmcpart 1; "\
                         "setenv mmcroot /dev/mmcblk0p1 rootwait ro; "\
                         "run loaduimage; "\
                         "run mmcboot; "\
                "fi; " \
        "fi;"

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#define CONFIG_LCD
	#define CONFIG_FB_BASE				(CONFIG_SYS_TEXT_BASE + 0x300000)
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#undef LCD_TEST_PATTERN
	#define CONFIG_SPLASH_IS_IN_MMC			1
	#define LCD_BPP					LCD_MONOCHROME
	#define CONFIG_SPLASH_SCREEN_ALIGN		1

	#define CONFIG_WORKING_BUF_ADDR			(CONFIG_SYS_TEXT_BASE + 0x100000)
	#define CONFIG_WAVEFORM_BUF_ADDR		(CONFIG_SYS_TEXT_BASE + 0x200000)
	#define CONFIG_WAVEFORM_FILE_OFFSET		0x600000
	#define CONFIG_WAVEFORM_FILE_SIZE		0xF0A00
	#define CONFIG_WAVEFORM_FILE_IN_MMC

#ifdef CONFIG_SPLASH_IS_IN_MMC
	#define CONFIG_SPLASH_IMG_OFFSET		0x4c000
	#define CONFIG_SPLASH_IMG_SIZE			0x19000
#endif
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */

#endif                         /* __MX6QSABRESD_CONFIG_H */
