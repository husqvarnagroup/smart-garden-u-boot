/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 Stefan Roese <sr@denx.de>
 */

#ifndef __CONFIG_LINKIT_SMART_7688_H
#define __CONFIG_LINKIT_SMART_7688_H

/* CPU */
#define CONFIG_SYS_MIPS_TIMER_FREQ	200000000

/* RAM */
#define CONFIG_SYS_SDRAM_BASE		0x80000000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_SYS_SDRAM_BASE + 0x100000

#define CONFIG_SYS_INIT_SP_OFFSET	0x400000

#ifdef CONFIG_BOOT_RAM
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

/* UART */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200, \
					  230400, 500000, 1500000 }

/* RAM */
#define CONFIG_SYS_MEMTEST_START	0x80100000
#define CONFIG_SYS_MEMTEST_END		0x80400000

/* Memory usage */
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_MALLOC_LEN		(1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)
#define CONFIG_SYS_CBSIZE		512

/* U-Boot */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

/* Environment settings */
#define CONFIG_ENV_OFFSET		0x80000
#define CONFIG_ENV_SIZE			(16 << 10)
#define CONFIG_ENV_SECT_SIZE		(64 << 10)

/*
 * Environment is right behind U-Boot in flash. Make sure U-Boot
 * doesn't grow into the environment area.
 */
#define CONFIG_BOARD_SIZE_LIMIT		CONFIG_ENV_OFFSET

#define CONFIG_EXTRA_ENV_SETTINGS				\
	"bootcmd=ping ${serverip};run net_nfs\0"		\
	"loadaddr=0x81000000\0"					\
	"tftpdir=linkit-smart-7688\0"				\
	"tftpram=tftp 80010000 ${tftpdir}/u-boot.bin;go 80010000\0" \
	"tt=run tftpram\0"					\
	"net_nfs=tftp 80a00000 /tftpboot/gardena/uImage;setenv rootpath /tftpboot/gardena/work/rootfs/yocto-2018-09-21;setenv bootargs console=ttyS2,${baudrate} root=/dev/nfs rw nfsroot=${serverip}:${rootpath},tcp,nfsvers=3 ip=${ipaddr}:${serverip}::${netmask}:${hostname}:eth0:off;bootm 80a00000\0"		\
	"load_uboot=tftp ${loadaddr} ${tftpdir}/u-boot.bin\0"	\
	"update_uboot=sf probe;"				\
		"sf update ${loadaddr} 0 ${filesize};saveenv\0"	\
	"upd_uboot=run load_uboot update_uboot\0"

#endif /* __CONFIG_LINKIT_SMART_7688_H */
