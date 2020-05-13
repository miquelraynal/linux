/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  NAND family Bad Block Management (BBM) header file
 *    - Bad Block Table (BBT) implementation
 *
 *  Copyright © 2005 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 *  Copyright © 2000-2005
 *  Thomas Gleixner <tglx@linuxtronix.de>
 */
#ifndef __LINUX_MTD_BBM_H
#define __LINUX_MTD_BBM_H

/* The maximum number of NAND chips in an array */
#define NAND_MAX_CHIPS		8

/**
 * struct nand_bbt_descr - bad block table descriptor
 * @options:	options for this descriptor
 * @pages:	the page(s) where we find the bbt, used with option BBT_ABSPAGE
 *		when bbt is searched, then we store the found bbts pages here.
 *		Its an array and supports up to 8 chips now
 * @offs:	offset of the pattern in the oob area of the page
 * @veroffs:	offset of the bbt version counter in the oob are of the page
 * @version:	version read from the bbt page during scan
 * @len:	length of the pattern, if 0 no pattern check is performed
 * @maxblocks:	maximum number of blocks to search for a bbt. This number of
 *		blocks is reserved at the end of the device where the tables are
 *		written.
 * @reserved_block_code: if non-0, this pattern denotes a reserved (rather than
 *              bad) block in the stored bbt
 * @pattern:	pattern to identify bad block table or factory marked good /
 *		bad blocks, can be NULL, if len = 0
 *
 * Descriptor for the bad block table marker and the descriptor for the
 * pattern which identifies good and bad blocks. The assumption is made
 * that the pattern and the version count are always located in the oob area
 * of the first block.
 */
struct nand_bbt_descr {
	int options;
	int pages[NAND_MAX_CHIPS];
	int offs;
	int veroffs;
	uint8_t version[NAND_MAX_CHIPS];
	int len;
	int maxblocks;
	int reserved_block_code;
	uint8_t *pattern;
};

/* Options for the bad block table descriptors */

/* The number of bits used per block in the bbt on the device */
#define NAND_BBT_1BIT		BIT(0)
#define NAND_BBT_2BIT		BIT(1)
#define NAND_BBT_4BIT		BIT(2)
#define NAND_BBT_8BIT		BIT(3)
#define NAND_BBT_NRBITS_MSK	(NAND_BBT_1BIT | NAND_BBT_2BIT | \
				 NAND_BBT_4BIT | NAND_BBT_8BIT)
/* The bad block table is in the last good block of the device */
#define NAND_BBT_LASTBLOCK	BIT(4)
/* The bbt is at the given page, else we must scan for the bbt */
#define NAND_BBT_ABSPAGE	BIT(5)
/* bbt is stored per chip on multichip devices */
#define NAND_BBT_PERCHIP	BIT(6)
/* bbt has a version counter at offset veroffs */
#define NAND_BBT_VERSION	BIT(7)
/* Create a bbt if none exists */
#define NAND_BBT_CREATE		BIT(8)
/*
 * Create an empty BBT with no vendor information. Vendor's information may be
 * unavailable, for example, if the NAND controller has a different data and OOB
 * layout or if this information is already purged. Must be used in conjunction
 * with NAND_BBT_CREATE.
 */
#define NAND_BBT_CREATE_EMPTY	BIT(9)
/* Write bbt if neccecary */
#define NAND_BBT_WRITE		BIT(10)
/* Read and write back block contents when writing bbt */
#define NAND_BBT_SAVECONTENT	BIT(11)
/*
 * Use a flash based bad block table. By default, OOB identifier is saved in
 * OOB area. This option is passed to the default bad block table function.
 */
#define NAND_BBT_USE_FLASH	BIT(12)
/*
 * Do not store flash based bad block table marker in the OOB area; store it
 * in-band.
 */
#define NAND_BBT_NO_OOB		BIT(13)
/*
 * Do not write new bad block markers to OOB; useful, e.g., when ECC covers
 * entire spare area. Must be used with NAND_BBT_USE_FLASH.
 */
#define NAND_BBT_NO_OOB_BBM	BIT(14)
/* This option skips the bbt scan during initialization. */
#define NAND_BBT_SKIP_SCAN	BIT(15)

/*
 * Flag set by nand_create_default_bbt_descr(), marking that the nand_bbt_descr
 * was allocated dynamicaly and must be freed in nand_release(). Has no meaning
 * in nand_chip.bbt_options.
 */
#define NAND_BBT_DYNAMICSTRUCT	BIT(31)

/* The maximum number of blocks to scan for a bbt */
#define NAND_BBT_SCAN_MAXBLOCKS	4

/*
 * Bad block scanning errors
 */
#define ONENAND_BBT_READ_ERROR		1
#define ONENAND_BBT_READ_ECC_ERROR	2
#define ONENAND_BBT_READ_FATAL_ERROR	4

/**
 * struct bbm_info - [GENERIC] Bad Block Table data structure
 * @bbt_erase_shift:	[INTERN] number of address bits in a bbt entry
 * @options:		options for this descriptor
 * @bbt:		[INTERN] bad block table pointer
 * @isbad_bbt:		function to determine if a block is bad
 * @badblock_pattern:	[REPLACEABLE] bad block scan pattern used for
 *			initial bad block scan
 * @priv:		[OPTIONAL] pointer to private bbm date
 */
struct bbm_info {
	int bbt_erase_shift;
	int options;

	uint8_t *bbt;

	int (*isbad_bbt)(struct mtd_info *mtd, loff_t ofs, int allowbbt);

	/* TODO Add more NAND specific fileds */
	struct nand_bbt_descr *badblock_pattern;

	void *priv;
};

/* OneNAND BBT interface */
extern int onenand_default_bbt(struct mtd_info *mtd);

#endif	/* __LINUX_MTD_BBM_H */
