// SPDX-License-Identifier:  GPL-2.0+
/*
 * mtd.c
 *
 * Generic command to handle basic operations on any memory device.
 *
 * Copyright: Bootlin, 2018
 * Author: Miquèl Raynal <miquel.raynal@bootlin.com>
 */

#include <command.h>
#include <common.h>
#include <console.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <malloc.h>
#include <mapmem.h>
#include <mtd.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>

static void mtd_dump_buf(u8 *buf, uint len, uint offset)
{
	int i, j;

	for (i = 0; i < len; ) {
		printf("0x%08x:\t", offset + i);
		for (j = 0; j < 8; j++)
			printf("%02x ", buf[i + j]);
		printf(" ");
		i += 8;
		for (j = 0; j < 8; j++)
			printf("%02x ", buf[i + j]);
		printf("\n");
		i += 8;
	}
}

static void mtd_show_device(struct mtd_info *mtd)
{
	/* Device */
	printf("* %s", mtd->name);
	if (mtd->dev)
		printf(" [device: %s] [parent: %s] [driver: %s]",
		       mtd->dev->name, mtd->dev->parent->name,
		       mtd->dev->driver->name);
	printf("\n");

	/* MTD information */
	printf("\t> type: ");
	switch (mtd->type) {
	case MTD_RAM:
		printf("RAM\n");
		break;
	case MTD_ROM:
		printf("ROM\n");
		break;
	case MTD_NORFLASH:
		printf("NOR flash\n");
		break;
	case MTD_NANDFLASH:
		printf("NAND flash\n");
		break;
	case MTD_DATAFLASH:
		printf("Data flash\n");
		break;
	case MTD_UBIVOLUME:
		printf("UBI volume\n");
		break;
	case MTD_MLCNANDFLASH:
		printf("MLC NAND flash\n");
		break;
	case MTD_ABSENT:
	default:
		printf("Unknown\n");
		break;
	}

	printf("\t> Size: 0x%llx bytes\n", mtd->size);
	printf("\t> Block: 0x%x bytes\n", mtd->erasesize);
	printf("\t> Min I/O: 0x%x bytes\n", mtd->writesize);

	if (mtd->oobsize) {
		printf("\t> OOB size: %u bytes\n", mtd->oobsize);
		printf("\t> OOB available: %u bytes\n", mtd->oobavail);
	}

	if (mtd->ecc_strength) {
		printf("\t> ECC strength: %u bits\n", mtd->ecc_strength);
		printf("\t> ECC step size: %u bytes\n", mtd->ecc_step_size);
		printf("\t> Bitflip threshold: %u bits\n",
		       mtd->bitflip_threshold);
	}
}

int mtd_probe_devices(void)
{
	const char *mtdparts = env_get("mtdparts");
	struct udevice *dev;
	int idx = 0;

	/* Probe devices with DM compliant drivers */
	while (!uclass_find_device(UCLASS_MTD, idx, &dev) && dev) {
		mtd_probe(dev);
		idx++;
	}

	/* Create the partitions defined in mtdparts, here the fun begins */

	/* Ignore the extra 'mtdparts=' prefix if any */
	if (strstr(mtdparts, "mtdparts="))
		mtdparts += 9;

	/* For each MTD device in mtdparts */
	while (mtdparts[0] != '\0') {
		struct mtd_partition *parts;
		struct mtd_info *parent;
		char mtdname[20];
		int mtdname_len, len, ret;

		mtdname_len = strchr(mtdparts, ':') - mtdparts;
		strncpy(mtdname, mtdparts, mtdname_len);
		mtdname[mtdname_len] = '\0';
		/* Move the pointer forward (including the ':') */
		mtdparts += mtdname_len + 1;
		parent = get_mtd_device_nm(mtdname);
		if (IS_ERR_OR_NULL(parent)) {
			printf("No device named %s\n", mtdname);
			return -EINVAL;
		}

		/*
		 * Parse the MTD device partitions. It will update the mtdparts
		 * pointer, create an array of parts (that must be freed), and
		 * return the number of partition structures in the array.
		 */
		ret = mtdparts_parse_part(parent, &mtdparts, &parts, &len);
		if (ret) {
			printf("Could not parse device %s\n", mtdname);
			return -EINVAL;
		}

		/*
		 * If there is already an MTD device named after the first
		 * partition name, this is probably because the registration
		 * already happened: don't add the partitions again.
		 */
		if (len && !IS_ERR_OR_NULL(get_mtd_device_nm(parts[0].name)))
			continue;

		add_mtd_partitions(parent, parts, len);
		free(parts);
	}

	return 0;
}

static uint mtd_len_to_pages(struct mtd_info *mtd, u64 len)
{
	do_div(len, mtd->writesize);

	return len;
}

static bool mtd_is_aligned_with_min_io_size(struct mtd_info *mtd, u64 size)
{
	return do_div(size, mtd->writesize);
}

static bool mtd_is_aligned_with_block_size(struct mtd_info *mtd, u64 size)
{
	return do_div(size, mtd->erasesize);
}

static int do_mtd_list(void)
{
	struct mtd_info *mtd;
	int dev_nb = 0;

	/* Ensure all devices (and their partitions) are probed */
	mtd_probe_devices();

	printf("List of MTD devices:\n");
	mtd_for_each_device(mtd) {
		mtd_show_device(mtd);
		dev_nb++;
	}

	if (!dev_nb) {
		printf("No MTD device found\n");
		return CMD_RET_FAILURE;
	}

	return 0;
}

static int do_mtd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct mtd_info *mtd;
	const char *cmd;
	char *mtdname;
	int ret;

	/* All MTD commands need at least two arguments */
	if (argc < 2)
		return CMD_RET_USAGE;

	/* Parse the command name and its optional suffixes */
	cmd = argv[1];

	/* List the MTD devices if that is what the user wants */
	if (strcmp(cmd, "list") == 0)
		return do_mtd_list();

	/*
	 * The remaining commands require also at least a device ID.
	 * Check the selected device is valid. Ensure it is probed.
	 */
	if (argc < 3)
		return CMD_RET_USAGE;

	mtdname = argv[2];
	mtd = get_mtd_device_nm(mtdname);
	if (IS_ERR_OR_NULL(mtd)) {
		mtd_probe_devices();
		mtd = get_mtd_device_nm(mtdname);
		if (IS_ERR_OR_NULL(mtd)) {
			printf("MTD device %s not found, ret %ld\n",
			       mtdname, PTR_ERR(mtd));
			return 1;
		}
	}

	argc -= 3;
	argv += 3;

	/* Do the parsing */
	if (!strncmp(cmd, "read", 4) || !strncmp(cmd, "dump", 4) ||
	    !strncmp(cmd, "write", 5)) {
		struct mtd_oob_ops io_op = {};
		bool dump, read, raw, woob;
		uint user_addr = 0;
		uint nb_pages;
		u8 *buf;
		u64 off, len, default_len;
		u32 oob_len;

		dump = !strncmp(cmd, "dump", 4);
		read = dump || !strncmp(cmd, "read", 4);
		raw = strstr(cmd, ".raw");
		woob = strstr(cmd, ".oob");

		if (!dump) {
			if (!argc)
				return CMD_RET_USAGE;

			user_addr = simple_strtoul(argv[0], NULL, 16);
			argc--;
			argv++;
		}

		default_len = dump ? mtd->writesize : mtd->size;
		off = argc > 0 ? simple_strtoul(argv[0], NULL, 16) : 0;
		len = argc > 1 ? simple_strtoul(argv[1], NULL, 16) :
				 default_len;
		nb_pages = mtd_len_to_pages(mtd, len);
		oob_len = woob ? nb_pages * mtd->oobsize : 0;

		if (mtd_is_aligned_with_min_io_size(mtd, off)) {
			printf("Offset not aligned with a page (0x%x)\n",
			       mtd->writesize);
			return CMD_RET_FAILURE;
		}

		if (mtd_is_aligned_with_min_io_size(mtd, len)) {
			printf("Size not a multiple of a page (0x%x)\n",
			       mtd->writesize);
			return CMD_RET_FAILURE;
		}

		if (dump)
			buf = kmalloc(len + oob_len, GFP_KERNEL);
		else
			buf = map_sysmem(user_addr, 0);

		if (!buf) {
			printf("Could not map/allocate the user buffer\n");
			return CMD_RET_FAILURE;
		}

		printf("%s %lldB (%d page(s)) at offset 0x%08llx%s%s\n",
		       read ? "Reading" : "Writing", len, nb_pages, off,
		       raw ? " [raw]" : "", woob ? " [oob]" : "");

		io_op.mode = raw ? MTD_OPS_RAW : MTD_OPS_AUTO_OOB;
		io_op.len = len;
		io_op.ooblen = oob_len;
		io_op.datbuf = buf;
		io_op.oobbuf = woob ? &buf[len] : NULL;

		if (read)
			ret = mtd_read_oob(mtd, off, &io_op);
		else
			ret = mtd_write_oob(mtd, off, &io_op);

		if (ret) {
			printf("%s on %s failed with error %d\n",
			       read ? "Read" : "Write", mtd->name, ret);
			return CMD_RET_FAILURE;
		}

		if (dump) {
			uint page;

			for (page = 0; page < nb_pages; page++) {
				u64 data_off = page * mtd->writesize;

				printf("\nDump %d data bytes from 0x%08llx:\n",
				       mtd->writesize, data_off);
				mtd_dump_buf(buf, mtd->writesize, data_off);

				if (woob) {
					u64 oob_off = page * mtd->oobsize;

					printf("Dump %d OOB bytes from page at 0x%08llx:\n",
					       mtd->oobsize, data_off);
					mtd_dump_buf(&buf[len + oob_off],
						     mtd->oobsize, 0);
				}
			}
		}

		if (dump)
			kfree(buf);
		else
			unmap_sysmem(buf);

	} else if (!strcmp(cmd, "erase")) {
		bool scrub = strstr(cmd, ".dontskipbad");
		struct erase_info erase_op = {};
		u64 off, len;

		off = argc > 0 ? simple_strtoul(argv[0], NULL, 16) : 0;
		len = argc > 1 ? simple_strtoul(argv[1], NULL, 16) : mtd->size;

		if (mtd_is_aligned_with_block_size(mtd, off)) {
			printf("Offset not aligned with a block (0x%x)\n",
			       mtd->erasesize);
			return CMD_RET_FAILURE;
		}

		if (mtd_is_aligned_with_block_size(mtd, len)) {
			printf("Size not a multiple of a block (0x%x)\n",
			       mtd->erasesize);
			return CMD_RET_FAILURE;
		}

		printf("Erasing 0x%08llx ... 0x%08llx (%d eraseblock(s))\n",
		       off, off + len - 1, mtd_div_by_eb(len, mtd));

		erase_op.mtd = mtd;
		erase_op.addr = off;
		erase_op.len = len;
		erase_op.scrub = scrub;

		while (erase_op.len) {
			ret = mtd_erase(mtd, &erase_op);

			/* Abort if its not an bad block error */
			if (ret != -EIO)
				break;

			printf("Skipping bad block at 0x%08llx\n",
			       erase_op.fail_addr);

			/* Skip bad block and continue behind it */
			erase_op.len -= erase_op.fail_addr - erase_op.addr;
			erase_op.len -= mtd->erasesize;
			erase_op.addr = erase_op.fail_addr + mtd->erasesize;
		}
	} else {
		return CMD_RET_USAGE;
	}

	if (ret)
		return CMD_RET_FAILURE;
	return 0;
}

static char mtd_help_text[] =
#ifdef CONFIG_SYS_LONGHELP
	"- generic operations on memory technology devices\n\n"
	"mtd list\n"
	"mtd read[.raw][.oob]    <name> <addr> [<off> [<size>]]\n"
	"mtd dump[.raw][.oob]    <name>        [<off> [<size>]]\n"
	"mtd write[.raw][.oob]   <name> <addr> [<off> [<size>]]\n"
	"mtd erase[.dontskipbad] <name>        [<off> [<size>]]\n"
	"\n"
	"With:\n"
	"\t<name>: NAND partition/chip name\n"
	"\t<addr>: user address from/to which data will be retrieved/stored\n"
	"\t<off>: offset in <name> in bytes (default: start of the part)\n"
	"\t\t* must be block-aligned for erase\n"
	"\t\t* must be page-aligned otherwise\n"
	"\t<size>: length of the operation in bytes (default: the entire device)\n"
	"\t\t* must be a multiple of a block for erase\n"
	"\t\t* must be a multiple of a page otherwise (special case: default is a page with dump)\n"
#endif
	"";

U_BOOT_CMD(mtd, 10, 1, do_mtd, "MTD utils", mtd_help_text);
