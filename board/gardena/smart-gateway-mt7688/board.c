// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Stefan Roese <sr@denx.de>
 */

#include <common.h>
#include <environment.h>
#include <led.h>
#include <net.h>
#include <spi.h>
#include <spi_flash.h>
#include <linux/io.h>

#define MT76XX_AGPIO_CFG	0x1000003c

#define OTP_OFFS		0x7ff000
#define OTP_SIZE		512
#define OTP_SECT_SIZE		0x1000

#define OTP_MAGIC		0xCAFEBABE

struct otp_values {
	u32 crc;
	u32 magic;
	u32 version;
	char ipr_id[38];	/* 37 bytes necessary -> lets use 38 bytes */
	char ethaddr[20];	/* 6*2 + 5*1 + 1 = 18 -> lets use 20 bytes */
};

int board_early_init_f(void)
{
	void __iomem *gpio_mode;

	/* Configure digital vs analog GPIOs */
	gpio_mode = ioremap_nocache(MT76XX_AGPIO_CFG, 0x100);
	iowrite32(0x00fe01ff, gpio_mode);

	return 0;
}

static int otp_env_restore_defaults(void)
{
	/* Clear the special env variables */
	env_set("ipr_id", '\0');
	env_set("ethaddr", '\0');

	/* Save the new values back into the environment */
	env_save();

	return 0;
}

static int otp_env_config(void)
{
	struct otp_values *otp;
	struct spi_flash *sf;
	u8 buf[OTP_SIZE];
	char *ipr_id;
	char *ethaddr;
	u32 crc;
	int ret;

	/*
	 * Get write-protected values from OTP area in SPI NOR
	 *
	 * 1.  First check if the OTP content is correct (magic, crc etc).
	 * 2a. If correct, then compare the values with the ones perhaps
	 *     already present in the environment.
	 * 3a. If identical, nothing needs to be done.
	 * 3b. If not, save the OTP values into the U-Boot environment
	 *     so that Linux can also use these values.
	 * 2b. If OTP values are not correct (e.g. wrong crc) print warning
	 *     and remove all currently configured special env variables
	 *     and save the enviroment.
	 */

	sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
			     CONFIG_SF_DEFAULT_CS,
			     CONFIG_SF_DEFAULT_SPEED,
			     CONFIG_SF_DEFAULT_MODE);
	if (!sf) {
		printf("OTP: Unable to access SPI NOR flash\n");
		goto err;
	}

	ret = spi_flash_read(sf, OTP_OFFS, OTP_SIZE, (void *)buf);
	if (ret) {
		printf("OTP: Unable to read OTP values from SPI NOR flash\n");
		goto err;
	}

	otp = (struct otp_values *)buf;

	crc = crc32(0, (u8 *)&otp->magic, OTP_SIZE - sizeof(u32));
	if (crc != otp->crc) {
		printf("OTP: CRC not correct\n");
		goto err;
	}

	/* Check if values are already identical in environment */
	ipr_id = env_get("ipr_id");
	ethaddr = env_get("ethaddr");

	if ((strcmp(ipr_id, otp->ipr_id)) ||
	    (strcmp(ethaddr, otp->ethaddr))) {
		printf("OTP:   OTP values don't match env values -> saving\n");
		env_set("ipr_id", otp->ipr_id);
		env_set("ethaddr", otp->ethaddr);
		env_save();
	} else {
		printf("OTP:   OTP values match current env values\n");
	}

	return 0;

err:
	otp_env_restore_defaults();
	return -ENODEV;
}

int board_late_init(void)
{
	if (IS_ENABLED(CONFIG_LED))
		led_default_state();

	otp_env_config();

	return 0;
}

#if 1 // test-only: remove once OTP is correctly programmed at board bringup
int do_otp_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct otp_values *otp;
	struct spi_flash *sf;
	u8 enetaddr[ARP_HLEN];
	u8 buf[OTP_SIZE];
	int ret;

	/* Generate the OTP struct */
	memset(buf, 0xff, OTP_SIZE);
	otp = (struct otp_values *)buf;
	otp->magic = OTP_MAGIC;
	otp->version = 100;

	net_random_ethaddr(enetaddr);
	memset(otp->ethaddr, 0x00, sizeof(otp->ethaddr));
	sprintf(otp->ethaddr, "%pM", enetaddr);

	// test-only: use ipd_id as parameter from console
	memset(otp->ipr_id, 0x00, sizeof(otp->ipr_id));
	snprintf(otp->ipr_id, sizeof(otp->ipr_id), "smart-gw-1234");

	printf("New OTP values:\n");
	printf("ethaddr=%s\n", otp->ethaddr);
	printf("ipr_id=%s\n", otp->ipr_id);

	otp->crc = crc32(0, (u8 *)&otp->magic, OTP_SIZE - sizeof(u32));

	sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
			     CONFIG_SF_DEFAULT_CS,
			     CONFIG_SF_DEFAULT_SPEED,
			     CONFIG_SF_DEFAULT_MODE);
	if (!sf) {
		printf("OTP: Unable to access SPI NOR flash\n");
		goto err;
	}

	ret = spi_flash_erase(sf, OTP_OFFS, OTP_SECT_SIZE);
	if (ret) {
		printf("OTP: spi_flash_erase failed (%d)\n", ret);
		goto err;
	}

	ret = spi_flash_write(sf, OTP_OFFS, OTP_SIZE, buf);
	if (ret) {
		printf("OTP: spi_flash_write failed (%d)\n", ret);
		goto err;
	}

	spi_flash_free(sf);
	printf("OTP values written to SPI NOR flash\n");

	return 0;

err:
	spi_flash_free(sf);

	return CMD_RET_FAILURE;
}

U_BOOT_CMD(
	otp_write,	1,	0,	do_otp_write,
	"Write OTP values to SPI NOR",
	"\n"
);
#endif
