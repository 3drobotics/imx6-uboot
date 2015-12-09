/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QSABRESD_CONFIG_H
#define __MX6QSABRESD_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_MMCROOT			"/dev/mmcblk2p2"

#include "mx6sabre_common.h"

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_GPIO
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1 /* Enabled USB controller number */

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC3 */
#define CONFIG_SYS_MMC_ENV_PART                0       /* user partition */

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   (0|(IMX_GPIO_NR(4, 9)<<8))
#endif

/*
 * imx6 q/dl/solo pcie would be failed to work properly in kernel, if
 * the pcie module is iniialized/enumerated both in uboot and linux
 * kernel.
 * rootcause:imx6 q/dl/solo pcie don't have the reset mechanism.
 * it is only be RESET by the POR. So, the pcie module only be
 * initialized/enumerated once in one POR.
 * Set to use pcie in kernel defaultly, mask the pcie config here.
 * Remove the mask freely, if the uboot pcie functions, rather than
 * the kernel's, are required.
 */
/* #define CONFIG_CMD_PCI */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif

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
        "zimage=zImage\0" \
        "fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
        "fdt_addr=0x18000000\0" \
        "boot_fdt=try\0" \
        "ip_dyn=yes\0" \
        "console=" CONFIG_CONSOLE_DEV "\0" \
        "fdt_high=0xffffffff\0"   \
        "initrd_high=0xffffffff\0" \
        "bootdelay=3\0" \
        "factoryReset=0\0" \
        "mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
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
        "loadzimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${zimage}\0" \
        "loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "mmcuboot=echo Booting from mmc ...; " \
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
        "mmczboot=echo Booting from mmc ...; " \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run loadfdt; then " \
                                "bootz ${loadaddr} - ${fdt_addr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootz; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootz; " \
                "fi;\0" \

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
        "mmc dev ${mmcdev};" \
        "if mmc rescan; then " \
                "if run loadbootscript; then " \
                "run bootscript; " \
                "else " \
                    "if test ${factoryReset} -eq 1; then " \
                         "setenv mmcpart 1; "\
                         "setenv mmcroot /dev/mmcblk0p1 rootwait ro; "\
                         "run loaduimage; "\
                         "run mmcuboot; "\
                    "fi; " \
                    "setenv mmcpart 2; "\
                    "setenv mmcroot /dev/mmcblk2p2 rootwait ro; "\
                        "if run loadzimage; then "\
                             "run mmczboot; " \
                         "fi; " \
                         "setenv mmcpart 1; "\
                         "setenv mmcroot /dev/mmcblk0p1 rootwait ro; "\
                         "run loaduimage; "\
                         "run mmcuboot; "\
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
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

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
