/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Sascha Hauer, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/iopoll.h>

#include <linux/platform_data/mtd-mxc_nand.h>

#define DRIVER_NAME "mxc_nand"
#define MXC_MAX_NAND_CHIPS 1

/* Addresses for NFC registers */
#define NFC_V1_V2_BUF_SIZE             (nfc->regs + 0x00)
#define NFC_V1_V2_BUF_ADDR             (nfc->regs + 0x04)
#define NFC_V1_V2_FLASH_ADDR           (nfc->regs + 0x06)
#define NFC_V1_V2_FLASH_CMD            (nfc->regs + 0x08)
#define NFC_V1_V2_CONFIG               (nfc->regs + 0x0a)
#define NFC_V1_V2_ECC_STATUS_RESULT    (nfc->regs + 0x0c)
#define NFC_V1_V2_RSLTMAIN_AREA                (nfc->regs + 0x0e)
#define NFC_V1_V2_RSLTSPARE_AREA       (nfc->regs + 0x10)
#define NFC_V1_V2_WRPROT               (nfc->regs + 0x12)
#define NFC_V1_UNLOCKSTART_BLKADDR     (nfc->regs + 0x14)
#define NFC_V1_UNLOCKEND_BLKADDR       (nfc->regs + 0x16)
#define NFC_V21_UNLOCKSTART_BLKADDR0   (nfc->regs + 0x20)
#define NFC_V21_UNLOCKSTART_BLKADDR1   (nfc->regs + 0x24)
#define NFC_V21_UNLOCKSTART_BLKADDR2   (nfc->regs + 0x28)
#define NFC_V21_UNLOCKSTART_BLKADDR3   (nfc->regs + 0x2c)
#define NFC_V21_UNLOCKEND_BLKADDR0     (nfc->regs + 0x22)
#define NFC_V21_UNLOCKEND_BLKADDR1     (nfc->regs + 0x26)
#define NFC_V21_UNLOCKEND_BLKADDR2     (nfc->regs + 0x2a)
#define NFC_V21_UNLOCKEND_BLKADDR3     (nfc->regs + 0x2e)
#define NFC_V1_V2_NF_WRPRST            (nfc->regs + 0x18)
#define NFC_V1_V2_CONFIG1              (nfc->regs + 0x1a)
#define NFC_V1_V2_CONFIG2              (nfc->regs + 0x1c)

#define NFC_V2_CONFIG1_ECC_MODE_4      (1 << 0)
#define NFC_V1_V2_CONFIG1_SP_EN                (1 << 2)
#define NFC_V1_V2_CONFIG1_ECC_EN       (1 << 3)
#define NFC_V1_V2_CONFIG1_INT_MSK      (1 << 4)
#define NFC_V1_V2_CONFIG1_BIG          (1 << 5)
#define NFC_V1_V2_CONFIG1_RST          (1 << 6)
#define NFC_V1_V2_CONFIG1_CE           (1 << 7)
#define NFC_V2_CONFIG1_ONE_CYCLE       (1 << 8)
#define NFC_V2_CONFIG1_PPB(x)          (((x) & 0x3) << 9)
#define NFC_V2_CONFIG1_FP_INT          (1 << 11)

#define NFC_V1_V2_CONFIG2_INT          (1 << 15)

/*
 * Operation modes for the NFC. Valid for v1, v2 and v3
 * type controllers.
 */
#define NFC_CMD                                (1 << 0)
#define NFC_ADDR                       (1 << 1)
#define NFC_INPUT                      (1 << 2)
#define NFC_OUTPUT                     (1 << 3)
#define NFC_ID                         (1 << 4)
#define NFC_STATUS                     (1 << 5)

#define NFC_V3_FLASH_CMD               (nfc->regs_axi + 0x00)
#define NFC_V3_FLASH_ADDR0             (nfc->regs_axi + 0x04)

#define NFC_V3_CONFIG1                 (nfc->regs_axi + 0x34)
#define NFC_V3_CONFIG1_SP_EN           (1 << 0)
#define NFC_V3_CONFIG1_RBA(x)          (((x) & 0x7) << 4)

#define NFC_V3_ECC_STATUS_RESULT       (nfc->regs_axi + 0x38)

#define NFC_V3_LAUNCH                  (nfc->regs_axi + 0x40)

#define NFC_V3_WRPROT                  (nfc->regs_ip + 0x0)
#define NFC_V3_WRPROT_LOCK_TIGHT       (1 << 0)
#define NFC_V3_WRPROT_LOCK             (1 << 1)
#define NFC_V3_WRPROT_UNLOCK           (1 << 2)
#define NFC_V3_WRPROT_BLS_UNLOCK       (2 << 6)

#define NFC_V3_WRPROT_UNLOCK_BLK_ADD0   (nfc->regs_ip + 0x04)

#define NFC_V3_CONFIG2                 (nfc->regs_ip + 0x24)
#define NFC_V3_CONFIG2_PS_512                  (0 << 0)
#define NFC_V3_CONFIG2_PS_2048                 (1 << 0)
#define NFC_V3_CONFIG2_PS_4096                 (2 << 0)
#define NFC_V3_CONFIG2_ONE_CYCLE               (1 << 2)
#define NFC_V3_CONFIG2_ECC_EN                  (1 << 3)
#define NFC_V3_CONFIG2_2CMD_PHASES             (1 << 4)
#define NFC_V3_CONFIG2_NUM_ADDR_PHASE0         (1 << 5)
#define NFC_V3_CONFIG2_ECC_MODE_8              (1 << 6)
#define NFC_V3_CONFIG2_PPB(x, shift)           (((x) & 0x3) << shift)
#define NFC_V3_CONFIG2_NUM_ADDR_PHASE1(x)      (((x) & 0x3) << 12)
#define NFC_V3_CONFIG2_INT_MSK                 (1 << 15)
#define NFC_V3_CONFIG2_ST_CMD(x)               (((x) & 0xff) << 24)
#define NFC_V3_CONFIG2_SPAS(x)                 (((x) & 0xff) << 16)

#define NFC_V3_CONFIG3                         (nfc->regs_ip + 0x28)
#define NFC_V3_CONFIG3_ADD_OP(x)               (((x) & 0x3) << 0)
#define NFC_V3_CONFIG3_FW8                     (1 << 3)
#define NFC_V3_CONFIG3_SBB(x)                  (((x) & 0x7) << 8)
#define NFC_V3_CONFIG3_NUM_OF_DEVICES(x)       (((x) & 0x7) << 12)
#define NFC_V3_CONFIG3_RBB_MODE                        (1 << 15)
#define NFC_V3_CONFIG3_NO_SDMA                 (1 << 20)

#define NFC_V3_IPC                     (nfc->regs_ip + 0x2C)
#define NFC_V3_IPC_CREQ                        (1 << 0)
#define NFC_V3_IPC_INT                 (1 << 31)

#define NFC_V3_DELAY_LINE              (nfc->regs_ip + 0x34)

struct mxc_nfc;

struct mxc_nand_devtype_data {
       bool (*get_int)(struct mxc_nfc *nfc);
       void (*irq_control)(struct mxc_nfc *nfc, int activate);
       int (*wait_op_done)(struct mxc_nfc *nfc, int useirq);
       void (*enable_hwecc)(struct mtd_info *nfc, int enable);
       void (*enable_spare_only)(struct mtd_info *nfc, int enable);
       int (*get_hwecc_status)(struct mxc_nfc *nfc);
       void (*preset)(struct mtd_info *mtd);
	int (*setup_data_interface)(struct mtd_info *mtd, int chipnr,
				    const struct nand_data_interface *conf);

       void (*send_cmd)(struct mxc_nfc *nfc, uint16_t cmd, int useirq);
       void (*send_addr)(struct mxc_nfc *nfc, uint16_t addr, int islast);
       void (*trig_data_copy)(struct mxc_nfc *nfc, int buffer,
                              int direction);
       int (*correct_data)(struct mtd_info *mtd);

       const struct mtd_ooblayout_ops *ooblayout;

       size_t regs_offset;
       size_t spare0_offset;
       size_t axi_offset;

       int spare_len;
       int eccbytes;
       int eccsize;
       int ppb_shift;
       int eccstrength;
       int prepad;
       int postpad;
};

/*
 * NAND chip structure: stores NAND chip device related information
 *
 * @nand:                       base NAND chip structure
 * @pdata:                      platform data as declared in mtd-mxc_nand.h
 */
struct mxc_nand_chip {
       struct nand_chip        nand;
       struct mxc_nand_platform_data pdata;
};

static inline struct mxc_nand_chip *to_mxc_chip(struct nand_chip *chip)
{
       return container_of(chip, struct mxc_nand_chip, nand);
}

/*
 * NAND controller structure: stores mxc NAND controller information
 *
 * MXC NAND controller has an internal SRAM buffer to store data during
 * read/write operations to/from the NAND chip. This page-sized buffer
 * is a container of N adjacent subpages followed by their respective OOB areas.
 * This entire memory region (counting also the following status and control
 * registers) is remapped and available through a 'base' pointer. Data may be
 * copied to/from SRAM with memcpy_toio/fromio() helpers into a RAM buffer
 * (subpage-sized, allocated by this driver) pointed to by 'data_buf'.
 * The SRAM layout is drawn below:
 *
 *    *main_area0                          *spare0                    *regs
 *  /  (== *base)                  (*base + spare_offset)  (*base + regs_offset)
 * |                                           |                          \
 * v                                           v                           `-->
 * .--------------------------- // ---------------------------- // --------.
 * |          |          |      //  |          |      |      |  //  |      |
 * '--------------------------- // ---------------------------- // --------'
 * ^          ^          ^          ^          ^      ^      ^      ^
 * |          |          |          |          |      |      |      |
 * |          |          |    Data subpage N   |      |      | OOB subpage N
 * |          |    Data subpage 3              |      | OOB subpage 3
 * |    Data subpage 2                         | OOB subpage 2
 * Data subpage 1                         OOB subpage 1
 *
 * @controller:                 base controller structure
 * @dev:                        parent device (used to print error messages
 *                              and browse device tree)
 * @base:                       remapped SRAM buffer
 * @main_area0:                 first main area (data) pointer
 * @spare0:                     first spare area (OOB) pointer
 * @regs:                       NAND controller registers pointer
 * @regs_axi                    AXI bus registers pointer
 * @regs_ip                     IP bus registers pointer
 * @clk:                        NAND controller clock
 * @clk_act:                    flag indicating NAND controller clock activation
 * @irq:                        NAND controller IRQ line
 * @op_completion:              for polling on end of operation
 * @data_buf:                   NAND controller driver buffer (data + OOB)
 * @buf_start:                  data_buf offset
 * @devtype_data:               NAND controller ECC engine parameters
 * @chips:                      array containing all the possible NAND chips
 *                              attached to this NAND controller
 */
struct mxc_nfc {
       struct nand_hw_control  controller;
       struct device           *dev;

       void __iomem            *base;
       void __iomem            *main_area0;
       void __iomem            *spare0;
       void __iomem            *regs;
       void __iomem            *regs_axi;
       void __iomem            *regs_ip;

       uint8_t                 *data_buf;
       int                     buf_start;

       struct clk              *clk;
       int                     clk_act;
       int                     irq;
       struct completion       op_completion;

       const struct mxc_nand_devtype_data *devtype_data;
       struct mxc_nand_chip    chips[MXC_MAX_NAND_CHIPS];
};

static inline struct mxc_nfc *to_mxc_nfc(struct nand_hw_control *ctrl)
{
       return container_of(ctrl, struct mxc_nfc, controller);
}

static const char * const part_probes[] = {
       "cmdlinepart", "RedBoot", "ofpart", NULL };

static inline bool is_imx21_nfc(struct mxc_nfc *nfc);
static inline bool is_imx27_nfc(struct mxc_nfc *nfc);
static inline bool is_imx25_nfc(struct mxc_nfc *nfc);
static inline bool is_imx51_nfc(struct mxc_nfc *nfc);
static inline bool is_imx53_nfc(struct mxc_nfc *nfc);

/*
 * Checks the presence of the IT bit from a config resiter, returns its value
 * and therefore the presence of an IT request.
 */
static bool get_int_imx21(struct mxc_nfc *nfc)
{
       uint16_t tmp;
       bool flag;

       tmp = readw(NFC_V1_V2_CONFIG2);
       flag = tmp & NFC_V1_V2_CONFIG2_INT;

       return flag;
}

static bool get_int_imx27_imx25(struct mxc_nfc *nfc)
{
       uint16_t tmp;
       bool flag;

       tmp = readw(NFC_V1_V2_CONFIG2);
       flag = tmp & NFC_V1_V2_CONFIG2_INT;
       if (flag)
               writew(tmp & ~NFC_V1_V2_CONFIG2_INT, NFC_V1_V2_CONFIG2);

       return flag;
}

static bool get_int_imx5x(struct mxc_nfc *nfc)
{
       uint16_t tmp;
       bool flag;

       tmp = readl(NFC_V3_IPC);
       flag = tmp & NFC_V3_IPC_INT;
       if (flag)
               writew(tmp & ~NFC_V3_IPC_INT, NFC_V3_IPC);

       return flag;
}

/* Activates or deactives the IT request bit from get_int() */
static void irq_control_imx21(struct mxc_nfc *nfc, int activate)
{
       if (activate)
               enable_irq(nfc->irq);
       else
               disable_irq_nosync(nfc->irq);
}

static void irq_control_imx27_imx25(struct mxc_nfc *nfc, int activate)
{
       uint16_t tmp;

       tmp = readw(NFC_V1_V2_CONFIG1);

       if (activate)
               tmp &= ~NFC_V1_V2_CONFIG1_INT_MSK;
       else
               tmp |= NFC_V1_V2_CONFIG1_INT_MSK;

       writew(tmp, NFC_V1_V2_CONFIG1);
}

static void irq_control_imx5x(struct mxc_nfc *nfc, int activate)
{
       uint16_t tmp;

       tmp = readw(NFC_V3_CONFIG2);

       if (activate)
               tmp &= ~NFC_V3_CONFIG2_INT_MSK;
       else
               tmp |= NFC_V3_CONFIG2_INT_MSK;

       writew(tmp, NFC_V3_CONFIG2);
}

static irqreturn_t nfc_irq(int irq, void *dev_id)
{
       struct mxc_nfc *nfc = dev_id;

       if (!nfc->devtype_data->get_int(nfc))
               return IRQ_NONE;

       nfc->devtype_data->irq_control(nfc, 0);

       complete(&nfc->op_completion);

       return IRQ_HANDLED;
}

/*
 * This function polls the NANDFC to wait for the basic operation to
 * complete by checking the INT bit of CONFIG2 register.
 */
static int wait_op_done_imx2x(struct mxc_nfc *nfc, int useirq)
{
       int ret = 0;

       /* If operation is already complete, don't setup an irq or a loop */
       if (nfc->devtype_data->get_int(nfc))
               return 0;

       if (useirq) {
               unsigned long timeout;

               reinit_completion(&nfc->op_completion);

               nfc->devtype_data->irq_control(nfc, 1);

               timeout = wait_for_completion_timeout(&nfc->op_completion, HZ);
               if (!timeout && !nfc->devtype_data->get_int(nfc)) {
                       dev_dbg(nfc->dev, "timeout waiting for irq\n");
                       ret = -ETIMEDOUT;
               }
       } else {
               uint16_t conf;

               ret = readw_poll_timeout_atomic(NFC_V1_V2_CONFIG2, conf,
                                               conf & NFC_V1_V2_CONFIG2_INT,
                                               1, 8000);
               if (ret < 0)
                       dev_warn(nfc->dev, "timeout polling for completion\n");
               else
                       writew(conf & ~NFC_V1_V2_CONFIG2_INT,
                              NFC_V1_V2_CONFIG2);
       }

       return ret;
}

static int wait_op_done_imx5x(struct mxc_nfc *nfc, int useirq)
{
       int ret = 0;

       /* If operation is already complete, don't setup an irq or a loop */
       if (nfc->devtype_data->get_int(nfc))
               return 0;

       if (useirq) {
               unsigned long timeout;

               reinit_completion(&nfc->op_completion);

               nfc->devtype_data->irq_control(nfc, 1);

               timeout = wait_for_completion_timeout(&nfc->op_completion, HZ);
               if (!timeout && !nfc->devtype_data->get_int(nfc)) {
                       dev_dbg(nfc->dev, "timeout waiting for irq\n");
                       ret = -ETIMEDOUT;
               }
       } else {
               uint16_t conf;

               ret = readl_poll_timeout_atomic(NFC_V3_IPC, conf,
                                               conf & NFC_V3_IPC_INT, 1, 8000);
               if (ret < 0)
                       dev_warn(nfc->dev, "timeout polling for completion\n");
               else
                       writel(conf & ~NFC_V3_IPC_INT, NFC_V3_IPC);
       }

       return ret;
}

/*
 * Enables/disables HW ECC operation. Only correction is canceled,
 * ECC status register is still relevant.
 */
static void enable_hwecc_imx2x(struct mtd_info *mtd, int enable)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint16_t config1;

       config1 = readw(NFC_V1_V2_CONFIG1);

       if (enable)
               config1 |= NFC_V1_V2_CONFIG1_ECC_EN;
       else
               config1 &= ~NFC_V1_V2_CONFIG1_ECC_EN;

       writew(config1, NFC_V1_V2_CONFIG1);
}

static void enable_hwecc_imx5x(struct mtd_info *mtd, int enable)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint16_t config2;

       config2 = readw(NFC_V3_CONFIG2);

       if (enable)
               config2 |= NFC_V3_CONFIG2_ECC_EN;
       else
               config2 &= ~NFC_V3_CONFIG2_ECC_EN;

       writel(config2, NFC_V3_CONFIG2);
}

/*
 * Enables/disables spare only reads/writes operation.
 */
static void enable_spare_only_imx2x(struct mtd_info *mtd, int enable)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint16_t config1;

       config1 = readw(NFC_V1_V2_CONFIG1);

       if (enable)
               config1 |= NFC_V1_V2_CONFIG1_SP_EN;
       else
               config1 &= ~NFC_V1_V2_CONFIG1_SP_EN;

       writew(config1, NFC_V1_V2_CONFIG1);
}

static void enable_spare_only_imx5x(struct mtd_info *mtd, int enable)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint16_t config1;

       config1 = readl(NFC_V3_CONFIG1);

       if (enable)
               config1 |= NFC_V3_CONFIG1_SP_EN;
       else
               config1 &= ~NFC_V3_CONFIG1_SP_EN;

       writel(config1, NFC_V3_CONFIG1);
}

/*
 * Reads the ECC status register (valid after a read or write operation,
 * even if hardware ECC engine is disabled).
 */
static int get_hwecc_status_imx21_imx27(struct mxc_nfc *nfc)
{
       return readw(NFC_V1_V2_ECC_STATUS_RESULT);
}

static int get_hwecc_status_imx25(struct mxc_nfc *nfc)
{
       return readl(NFC_V1_V2_ECC_STATUS_RESULT);
}

static int get_hwecc_status_imx5x(struct mxc_nfc *nfc)
{
       return readl(NFC_V3_ECC_STATUS_RESULT);
}

/*
 * imx25 and imx5x type controllers can do 4bit or 8bit ecc depending
 * on how much oob the nand chip has. For 8bit ecc we need at least
 * 26 bytes of oob data per 512 byte block.
 */
static int get_eccsize(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       int oobbytes_per_512 = 0;

       if (is_imx21_nfc(nfc) || is_imx27_nfc(nfc))
               return nfc->devtype_data->eccsize;

       oobbytes_per_512 = mtd->oobsize * 512 / mtd->writesize;

       if (oobbytes_per_512 < 26)
               return 4;
       else
               return 8;
}

static void preset_imx21(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       /* Blocks to be unlocked */
       writew(0x0, NFC_V1_UNLOCKSTART_BLKADDR);
       writew(0xffff, NFC_V1_UNLOCKEND_BLKADDR);

       /* Unlock Block Command for given address range */
       writew(0x4, NFC_V1_V2_WRPROT);

       /* HWECC only enabled when reading a page in hw mode */
       nfc->devtype_data->enable_hwecc(mtd, false);
}

static void preset_imx27(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       /* Mask interrupts */
       writew(NFC_V1_V2_CONFIG1_INT_MSK, NFC_V1_V2_CONFIG1);

       /* Blocks to be unlocked */
       writew(0x0, NFC_V1_UNLOCKSTART_BLKADDR);
       writew(0xffff, NFC_V1_UNLOCKEND_BLKADDR);

       /* Unlock Block Command for given address range */
       writew(0x4, NFC_V1_V2_WRPROT);

       /* HWECC only enabled when reading a page in hw mode */
       nfc->devtype_data->enable_hwecc(mtd, false);
}

static void preset_imx25(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint16_t pages_per_block = mtd->erasesize / mtd->writesize;
       uint16_t config1;

       /* Mask interrupts */
       config1 = NFC_V2_CONFIG1_FP_INT | NFC_V1_V2_CONFIG1_INT_MSK;
       config1 |= NFC_V2_CONFIG1_PPB(ffs(pages_per_block) - 6);
       if (nand->ecc.strength == 4)
               config1 |= NFC_V2_CONFIG1_ECC_MODE_4;
       writew(config1, NFC_V1_V2_CONFIG1);

       /* Blocks to be unlocked */
       writew(0x0, NFC_V21_UNLOCKSTART_BLKADDR0);
       writew(0x0, NFC_V21_UNLOCKSTART_BLKADDR1);
       writew(0x0, NFC_V21_UNLOCKSTART_BLKADDR2);
       writew(0x0, NFC_V21_UNLOCKSTART_BLKADDR3);
       writew(0xffff, NFC_V21_UNLOCKEND_BLKADDR0);
       writew(0xffff, NFC_V21_UNLOCKEND_BLKADDR1);
       writew(0xffff, NFC_V21_UNLOCKEND_BLKADDR2);
       writew(0xffff, NFC_V21_UNLOCKEND_BLKADDR3);

       /* Unlock Block Command for given address range */
       writew(0x4, NFC_V1_V2_WRPROT);

       /* HWECC only enabled when reading a page in hw mode */
       nfc->devtype_data->enable_hwecc(mtd, false);
}

static void preset_imx5x(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint32_t config2, config3;
       int i, addr_phases;

       writel(NFC_V3_CONFIG1_RBA(0), NFC_V3_CONFIG1);
       writel(NFC_V3_IPC_CREQ, NFC_V3_IPC);

       /* Blocks to be unlocked */
       for (i = 0; i < NAND_MAX_CHIPS; i++)
               writel(0xffff << 16, NFC_V3_WRPROT_UNLOCK_BLK_ADD0 + (i << 2));

       writel(0, NFC_V3_IPC);

       config2 = NFC_V3_CONFIG2_ONE_CYCLE |
               NFC_V3_CONFIG2_2CMD_PHASES |
               NFC_V3_CONFIG2_SPAS(mtd->oobsize >> 1) |
               NFC_V3_CONFIG2_ST_CMD(0x70) |
               NFC_V3_CONFIG2_INT_MSK |
               NFC_V3_CONFIG2_NUM_ADDR_PHASE0;

       addr_phases = fls(nand->pagemask) >> 3;

       if (mtd->writesize == 2048) {
               config2 |= NFC_V3_CONFIG2_PS_2048;
               config2 |= NFC_V3_CONFIG2_NUM_ADDR_PHASE1(addr_phases);
       } else if (mtd->writesize == 4096) {
               config2 |= NFC_V3_CONFIG2_PS_4096;
               config2 |= NFC_V3_CONFIG2_NUM_ADDR_PHASE1(addr_phases);
       } else {
               config2 |= NFC_V3_CONFIG2_PS_512;
               config2 |= NFC_V3_CONFIG2_NUM_ADDR_PHASE1(addr_phases - 1);
       }

       if (mtd->writesize) {
               if (nand->ecc.mode == NAND_ECC_HW)
                       config2 |= NFC_V3_CONFIG2_ECC_EN;

               config2 |= NFC_V3_CONFIG2_PPB(
                       ffs(mtd->erasesize / mtd->writesize) - 6,
                       nfc->devtype_data->ppb_shift);
               if (get_eccsize(mtd))
                       config2 |= NFC_V3_CONFIG2_ECC_MODE_8;
       }

       writel(config2, NFC_V3_CONFIG2);

       config3 = NFC_V3_CONFIG3_NUM_OF_DEVICES(0) |
               NFC_V3_CONFIG3_NO_SDMA |
               NFC_V3_CONFIG3_RBB_MODE |
               NFC_V3_CONFIG3_SBB(6) | /* Reset default */
               NFC_V3_CONFIG3_ADD_OP(0);

       if (!(nand->options & NAND_BUSWIDTH_16))
               config3 |= NFC_V3_CONFIG3_FW8;

       writel(config3, NFC_V3_CONFIG3);

       writel(0, NFC_V3_DELAY_LINE);
}

static int setup_data_interface_imx25(struct mtd_info *mtd, int chipnr,
				      const struct nand_data_interface *conf)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       int tRC_min_ns, tRC_ps, ret;
       unsigned long rate, rate_round;
       const struct nand_sdr_timings *timings;
       u16 config1;

       timings = nand_get_sdr_timings(conf);
       if (IS_ERR(timings))
               return -ENOTSUPP;

       config1 = readw(NFC_V1_V2_CONFIG1);

       tRC_min_ns = timings->tRC_min / 1000;
       rate = 1000000000 / tRC_min_ns;

       /*
        * For tRC < 30ns we have to use EDO mode. In this case the controller
        * does one access per clock cycle. Otherwise the controller does one
        * access in two clock cycles, thus we have to double the rate to the
        * controller.
        */
       if (tRC_min_ns < 30) {
               rate_round = clk_round_rate(nfc->clk, rate);
               config1 |= NFC_V2_CONFIG1_ONE_CYCLE;
               tRC_ps = 1000000000 / (rate_round / 1000);
       } else {
               rate *= 2;
               rate_round = clk_round_rate(nfc->clk, rate);
               config1 &= ~NFC_V2_CONFIG1_ONE_CYCLE;
               tRC_ps = 1000000000 / (rate_round / 1000 / 2);
       }

       /*
        * The timing values compared against are from the i.MX25 Automotive
        * datasheet, Table 50. NFC Timing Parameters
        */
       if (timings->tCLS_min > tRC_ps - 1000 ||
           timings->tCLH_min > tRC_ps - 2000 ||
           timings->tCS_min > tRC_ps - 1000 ||
           timings->tCH_min > tRC_ps - 2000 ||
           timings->tWP_min > tRC_ps - 1500 ||
           timings->tALS_min > tRC_ps ||
           timings->tALH_min > tRC_ps - 3000 ||
           timings->tDS_min > tRC_ps ||
           timings->tDH_min > tRC_ps - 5000 ||
           timings->tWC_min > 2 * tRC_ps ||
           timings->tWH_min > tRC_ps - 2500 ||
           timings->tRR_min > 6 * tRC_ps ||
           timings->tRP_min > 3 * tRC_ps / 2 ||
           timings->tRC_min > 2 * tRC_ps ||
           timings->tREH_min > (tRC_ps / 2) - 2500) {
               dev_dbg(nfc->dev, "Timing out of bounds\n");
               return -EINVAL;
       }

       if (chipnr < 0)
               return 0;

       ret = clk_set_rate(nfc->clk, rate);
       if (ret)
               return ret;

       writew(config1, NFC_V1_V2_CONFIG1);

       dev_dbg(nfc->dev, "Setting rate to %ldHz, %s mode\n", rate_round,
               config1 & NFC_V2_CONFIG1_ONE_CYCLE ? "One cycle (EDO)" :
               "normal");

       return 0;
}

/* Issues the specified command to the NAND device and waits for completion. */
static void send_cmd_imx2x(struct mxc_nfc *nfc, uint16_t cmd, int useirq)
{
       pr_debug("send_cmd(nfc, 0x%x, %d)\n", cmd, useirq);

       writew(cmd, NFC_V1_V2_FLASH_CMD);
       writew(NFC_CMD, NFC_V1_V2_CONFIG2);

       nfc->devtype_data->wait_op_done(nfc, useirq);
}

static void send_cmd_imx5x(struct mxc_nfc *nfc, uint16_t cmd, int useirq)
{
       pr_debug("send_cmd(nfc, 0x%x, %d)\n", cmd, useirq);

       writel(cmd, NFC_V3_FLASH_CMD);
       writew(NFC_CMD, NFC_V3_LAUNCH);

       nfc->devtype_data->wait_op_done(nfc, useirq);
}

/*
 * Sends an address (or partial address) to the NAND device.
 * The address is used to select the source/destination for a NAND command.
 */
static void send_addr_imx2x(struct mxc_nfc *nfc, uint16_t addr,
                     int islast)
{
       pr_debug("send_addr(nfc, 0x%x %d)\n", addr, islast);

       writew(addr, NFC_V1_V2_FLASH_ADDR);
       writew(NFC_ADDR, NFC_V1_V2_CONFIG2);

       nfc->devtype_data->wait_op_done(nfc, islast);
}

static void send_addr_imx5x(struct mxc_nfc *nfc, uint16_t addr,
                           int islast)
{
       pr_debug("send_addr(nfc, 0x%x %d)\n", addr, islast);

       writew(addr, NFC_V3_FLASH_ADDR0);
       writew(NFC_ADDR, NFC_V3_LAUNCH);

       nfc->devtype_data->wait_op_done(nfc, islast);
}

/*
 * Triggers a data copy between SRAM and NAND chip (blocking until operation
 * is completed)
 */
static inline void trig_data_copy_imx2x(struct mxc_nfc *nfc,
                                       int buffer, int direction)
{
       writew(buffer, NFC_V1_V2_BUF_ADDR);
       writew(direction, NFC_V1_V2_CONFIG2);

       nfc->devtype_data->wait_op_done(nfc, true);
}

static inline void trig_data_copy_imx5x(struct mxc_nfc *nfc,
                                       int buffer, int direction)
{
       uint32_t tmp;

       /* Reset RBA (RAM Buffer Address) value (bits 4-6 of CONFIG1) */
       tmp = readl(NFC_V3_CONFIG1);
       tmp &= ~(7 << 4);
       /* Select proper buffer */
       tmp |= (buffer << 4);
       writel(tmp, NFC_V3_CONFIG1);

       writel(direction, NFC_V3_LAUNCH);

       nfc->devtype_data->wait_op_done(nfc, false);
}

/*
 * This function is used by upper layer for select and
 * deselect of the NAND chip
 */
static void mxc_nfc_select_chip(struct mtd_info *mtd, int chip)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       if (chip > 0 && chip >= MXC_MAX_NAND_CHIPS)
               return;

       if (chip == -1) {
               /* Disable the NFC clock */
               if (nfc->clk_act) {
                       clk_disable_unprepare(nfc->clk);
                       nfc->clk_act = 0;
               }
               return;
       }

       if (!nfc->clk_act) {
               /* Enable the NFC clock */
               clk_prepare_enable(nfc->clk);
               nfc->clk_act = 1;
       }

       nfc->controller.active = &nfc->chips[chip].nand;

       if (is_imx25_nfc(nfc))
               writew(chip << 4, NFC_V1_V2_BUF_ADDR);
}

/*
 * This function is not used by upper layer, it is called after a subpage read
 * to give notice of the number of bitflips by its return value:
 *     0:        0  bitflip
 *     1:        1  bitflip  (corrected)
 *     -EBADMSG: 2+ bitflips (uncorrectable)
 * Please note that the returned value is only valid for the last subpage read !
 */
static int correct_data_imx21_imx27(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       struct device *dev = mtd->dev.parent;
       int bitflips = 0;

       /*
        * 1-Bit errors (data or OOB) are automatically corrected in HW. No need
        * for additional correction. 2-Bit errors or more cannot be corrected
        * by HW ECC, so we need to return failure.
        */
       uint16_t ecc_status = nfc->devtype_data->get_hwecc_status(nfc);

       if (((ecc_status & 0x3) == 2) || ((ecc_status >> 2) == 2)) {
               dev_dbg(dev, "HWECC uncorrectable 2-bit+ ECC error\n");
               bitflips = -EBADMSG;
       } else if (ecc_status) {
               dev_dbg(dev, "HWECC corrected %d bit(s)\n", bitflips);
               bitflips = (ecc_status & 0x1) + ((ecc_status >> 2) & 0x1);
       }

       return bitflips;
}

/*
 * This function is not used by upper layer, it is called after a subpage read
 * to give notice of the number of bitflips by its return value (depending on
 * the number of bits for ECC mode, 4 or 8):
 *     0:        0  bitflip
 *     1-[4|8]:  Number of bitflips (corrected)
 *     -EBADMSG: More than [4|8] bitflips (uncorrectable)
 * Please note that the returned value is only valid for the last subpage read !
 */
static int correct_data_imx25_imx5x(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       struct device *dev = mtd->dev.parent;
       int bitflips = 0;
       uint8_t ecc_bit_mask = (get_eccsize(mtd) == 4) ? 0x7 : 0xf;
       uint8_t bf_limit = (get_eccsize(mtd) == 4) ? 0x4 : 0x8;
       uint32_t ecc_errors = nfc->devtype_data->get_hwecc_status(nfc)
               & ecc_bit_mask;

       if (ecc_errors > bf_limit) {
               dev_dbg(dev, "Uncorrectable RS-ECC error\n");
               bitflips = -EBADMSG;
       } else {
               dev_dbg(dev, "%d correctable RS-ECC error\n", ecc_errors);
               bitflips = ecc_errors;
       }

       return bitflips;
}

static void mxc_nfc_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       nfc->buf_start = 0;

       if (ctrl & NAND_CLE)
               nfc->devtype_data->send_cmd(nfc, dat, false);

       if (ctrl & NAND_ALE)
               nfc->devtype_data->send_addr(nfc, dat, false);
}

static u_char mxc_nfc_read_byte(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);
       uint8_t ret;

       nand->read_buf(mtd, &ret, 1);

       if (nand->options & NAND_BUSWIDTH_16)
               nfc->buf_start++;

       return ret;
}

static uint16_t mxc_nfc_read_word(struct mtd_info *mtd)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       uint16_t ret;

       nand->read_buf(mtd, (uint8_t *)&ret, 2);

       return ret;
}

/*
 * Fetch data from the NAND Flash, then read it from SRAM.
 * During a READ0 operation, NFC fetches data from NAND cache and changes its
 * organization from a syndrome layout (data and OOB are interlaced) to a
 * more readable flat layout (all the data and then all the OOB).
 * Because operations like COMMANDS needs the in-chip layout, we must
 * reconstruct the page.
 */
static void mxc_nfc_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       int data_size = nfc->devtype_data->eccstrength;
       int spare_size = nfc->devtype_data->spare_len;
       int subpage_size = data_size + spare_size;
       int size;

       while (len) {
               /* Each write operation covers a subpage */
               if (nfc->buf_start >= subpage_size)
                       nfc->buf_start = 0;

               /*
                * Load in NAND cache and fetch a buffer at each start
                * of a subpage.
                */
               if (nfc->buf_start == 0) {
                       nfc->devtype_data->trig_data_copy(nfc, 0, NFC_OUTPUT);

                       memcpy_fromio(
                               nfc->data_buf + nfc->buf_start,
                               nfc->main_area0,
                               data_size);
                       memcpy_fromio(
                               nfc->data_buf + nfc->buf_start + data_size,
                               nfc->spare0,
                               spare_size);
               }

               /* Loop everytime an operation reaches a subpage_size stone */
               size = min(len, subpage_size - nfc->buf_start);
               memcpy(buf, nfc->data_buf + nfc->buf_start, size);

               len -= size;
               buf += size;
               nfc->buf_start += size;
       }
}

/*
 * Write data of length len to buffer buf. The data to be written on NAND Flash
 * is copied to SRAM. Once a subpage (data + OOB) has been copied, it is
 * written to NAND Flash.
 */
static void mxc_nfc_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
       struct nand_chip *nand = mtd_to_nand(mtd);
       struct mxc_nfc *nfc = to_mxc_nfc(nand->controller);

       int data_size = nfc->devtype_data->eccstrength;
       int spare_size = nfc->devtype_data->spare_len;
       int subpage_size = data_size + spare_size;
       int size;

       while (len) {
               /* Cannot address over a fullpage_size number of bytes */
               if (nfc->buf_start >= subpage_size)
                       nfc->buf_start = 0;

               /* Loop everytime an operation reaches a subpage_size stone */
               size = min(len, subpage_size - (nfc->buf_start));
               memcpy(nfc->data_buf + nfc->buf_start, buf, size);

               /*
                * When we reach the end of a subpage or the remaining data
                * is not aligned on a supage : send the buffer to NAND cache
                * then write it in the chip.
                */
               if ((nfc->buf_start + size) % subpage_size == 0) {
                       memcpy_toio(
                               nfc->main_area0,
                               nfc->data_buf,
                               data_size);
                       memcpy_toio(
                               nfc->spare0,
                               nfc->data_buf + data_size,
                               spare_size);

                       nfc->devtype_data->trig_data_copy(nfc, 0, NFC_INPUT);
               }

               len -= size;
               buf += size;
               nfc->buf_start += size;
       }
}

static int mxc_nfc_read_page_hwecc(struct mtd_info *mtd,
                                   struct nand_chip *chip, uint8_t *buf,
                                   int oob_required, int page)
{
       struct mxc_nfc *nfc = to_mxc_nfc(chip->controller);
       int bf, max_bf = 0;
       uint8_t *p = buf;
       int i;

       nfc->devtype_data->enable_hwecc(mtd, true);

       /* Load page in NAND chip cache */
       chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

       for (i = 0; i < chip->ecc.steps; i++, p += chip->ecc.size) {
               /* Fetch a subpage */
               nfc->devtype_data->trig_data_copy(nfc, i, NFC_OUTPUT);

               /* Copy the subpage to the given buffer */
               memcpy_fromio(p, nfc->main_area0 + (i * chip->ecc.size),
                             chip->ecc.size);

               /* Check the status register */
               bf = nfc->devtype_data->correct_data(mtd);
               if (bf < 0) {
                       mtd->ecc_stats.failed++;
               } else if (bf > 0) {
                       mtd->ecc_stats.corrected += bf;
                       max_bf = max_t(unsigned int, max_bf, bf);
               }
       }

       /* NFC concatenates OOB at the end of SRAM buffer */
       if (oob_required)
               memcpy_fromio(chip->oob_poi, nfc->spare0, mtd->oobsize);

       nfc->devtype_data->enable_hwecc(mtd, false);

       return max_bf;
}

static int mxc_nfc_read_oob_raw_hwecc(struct mtd_info *mtd,
                                      struct nand_chip *chip, int page)
{
       struct mxc_nfc *nfc = to_mxc_nfc(chip->controller);
       int spare_len = nfc->devtype_data->spare_len;
       int i, step;

       /*
        * Spare only operation means the data pointed to by RNDOUT command
        * will be available nicely adjusted by the NFC at the end of
        * the buffer.
        */
       nfc->devtype_data->enable_spare_only(mtd, true);

       for (i = 0, step = 1; i < chip->ecc.steps; i++, step++) {
               /* Load OOB area in cache */
               chip->cmdfunc(mtd, NAND_CMD_RNDOUT, (step * chip->ecc.size)
                             + (i * spare_len), -1);

               /* Fetch the OOB data */
               nfc->devtype_data->trig_data_copy(nfc, i, NFC_OUTPUT);
       }

       /* Copy OOB data available at the end of the SRAM buffer */
       memcpy_fromio(chip->oob_poi, nfc->spare0, mtd->oobsize);

       nfc->devtype_data->enable_spare_only(mtd, false);

       return 0;
}

static int mxc_nfc_read_oob_hwecc(struct mtd_info *mtd,
                                  struct nand_chip *chip, int page)
{
       struct mxc_nfc *nfc = to_mxc_nfc(chip->controller);

       nfc->devtype_data->enable_hwecc(mtd, true);

       mxc_nfc_read_oob_raw_hwecc(mtd, chip, page);

       nfc->devtype_data->enable_hwecc(mtd, false);

       return 0;
}

static int mxc_nfc_write_page_hwecc(struct mtd_info *mtd,
                                   struct nand_chip *chip,
                                   const uint8_t *buf, int oob_required,
                                   int page)
{
       struct mxc_nfc *nfc = to_mxc_nfc(chip->controller);
       int i;

       nfc->devtype_data->enable_hwecc(mtd, true);

       /* Copy buffers to SRAM */
       memcpy_toio(nfc->main_area0, buf, mtd->writesize);
       memcpy_toio(nfc->spare0, chip->oob_poi, mtd->oobsize);

       /* Send data to NAND cache */
       for (i = 0; i < chip->ecc.steps; i++)
               nfc->devtype_data->trig_data_copy(nfc, i, NFC_INPUT);

       nfc->devtype_data->enable_hwecc(mtd, false);

       return 0;
}

static int ooblayout_ecc_imx21_imx27(struct mtd_info *mtd, int section,
                                   struct mtd_oob_region *oobregion)
{
       struct nand_chip *nand = mtd_to_nand(mtd);

       if (section >= nand->ecc.steps)
               return -ERANGE;

       oobregion->offset = (section * 16) + 6;
       oobregion->length = nand->ecc.bytes;

       return 0;
}

static int ooblayout_free_imx21_imx27(struct mtd_info *mtd, int section,
                                     struct mtd_oob_region *oobregion)
{
       struct nand_chip *nand = mtd_to_nand(mtd);

       if (section > nand->ecc.steps)
               return -ERANGE;

       if (!section) {
               if (mtd->writesize <= 512) {
                       oobregion->offset = 0;
                       oobregion->length = 5;
               } else {
                       oobregion->offset = 2;
                       oobregion->length = 4;
               }
       } else {
               oobregion->offset = ((section - 1) * 16) +
                       nand->ecc.bytes + 6;
               if (section < nand->ecc.steps)
                       oobregion->length = (section * 16) + 6 -
                               oobregion->offset;
               else
                       oobregion->length = mtd->oobsize - oobregion->offset;
       }

       return 0;
}

static const struct mtd_ooblayout_ops ooblayout_ops_imx21_imx27 = {
       .ecc = ooblayout_ecc_imx21_imx27,
       .free = ooblayout_free_imx21_imx27,
};

static int ooblayout_ecc_imx25_imx5x(struct mtd_info *mtd, int section,
                                    struct mtd_oob_region *oobregion)
{
       struct nand_chip *nand_chip = mtd_to_nand(mtd);
       int stepsize = nand_chip->ecc.bytes == 9 ? 16 : 26;

       if (section >= nand_chip->ecc.steps)
               return -ERANGE;

       oobregion->offset = (section * stepsize) + 7;
       oobregion->length = nand_chip->ecc.bytes;

       return 0;
}

static int ooblayout_free_imx25_imx5x(struct mtd_info *mtd, int section,
                                     struct mtd_oob_region *oobregion)
{
       struct nand_chip *nand_chip = mtd_to_nand(mtd);
       int stepsize = nand_chip->ecc.bytes == 9 ? 16 : 26;

       if (section >= nand_chip->ecc.steps)
               return -ERANGE;

       if (!section) {
               if (mtd->writesize <= 512) {
                       oobregion->offset = 0;
                       oobregion->length = 5;
               } else {
                       oobregion->offset = 2;
                       oobregion->length = 4;
               }
       } else {
               oobregion->offset = section * stepsize;
               oobregion->length = 7;
       }

       return 0;
}

static const struct mtd_ooblayout_ops ooblayout_ops_imx25_imx5x = {
       .ecc = ooblayout_ecc_imx25_imx5x,
       .free = ooblayout_free_imx25_imx5x,
};

/*
 * The generic flash bbt decriptors overlap with our ecc
 * hardware, so define some i.MX specific ones.
 */
static uint8_t bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
       .options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
           | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
       .offs = 0,
       .len = 4,
       .veroffs = 4,
       .maxblocks = 4,
       .pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
       .options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
           | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
       .offs = 0,
       .len = 4,
       .veroffs = 4,
       .maxblocks = 4,
       .pattern = mirror_pattern,
};

/* v1 + irqpending_quirk: i.MX21 */
static const struct mxc_nand_devtype_data imx21_nand_devtype_data = {
       .get_int = get_int_imx21,
       .irq_control = irq_control_imx21,
       .wait_op_done = wait_op_done_imx2x,
       .enable_hwecc = enable_hwecc_imx2x,
       .enable_spare_only = enable_spare_only_imx2x,
       .get_hwecc_status = get_hwecc_status_imx21_imx27,
       .preset = preset_imx21,
       .setup_data_interface = NULL,
       .send_cmd = send_cmd_imx2x,
       .send_addr = send_addr_imx2x,
       .trig_data_copy = trig_data_copy_imx2x,
       .correct_data = correct_data_imx21_imx27,
       .ooblayout = &ooblayout_ops_imx21_imx27,
       .regs_offset = 0xe00,
       .spare0_offset = 0x800,
       .spare_len = 16,
       .eccbytes = 3,
       .eccsize = 1,
       .eccstrength = 512,
       .prepad = 6,
       .postpad = 7,
};

/* v1 + !irqpending_quirk: i.MX27, i.MX31 */
static const struct mxc_nand_devtype_data imx27_nand_devtype_data = {
       .get_int = get_int_imx27_imx25,
       .irq_control = irq_control_imx27_imx25,
       .wait_op_done = wait_op_done_imx2x,
       .enable_hwecc = enable_hwecc_imx2x,
       .enable_spare_only = enable_spare_only_imx2x,
       .get_hwecc_status = get_hwecc_status_imx21_imx27,
       .preset = preset_imx27,
       .setup_data_interface = NULL,
       .send_cmd = send_cmd_imx2x,
       .send_addr = send_addr_imx2x,
       .trig_data_copy = trig_data_copy_imx2x,
       .correct_data = correct_data_imx21_imx27,
       .ooblayout = &ooblayout_ops_imx21_imx27,
       .regs_offset = 0xe00,
       .spare0_offset = 0x800,
       .spare_len = 16,
       .eccbytes = 3,
       .eccsize = 1,
       .eccstrength = 512,
       .prepad = 6,
       .postpad = 7,
};

/* v21: i.MX25, i.MX35 */
static const struct mxc_nand_devtype_data imx25_nand_devtype_data = {
       .get_int = get_int_imx27_imx25,
       .irq_control = irq_control_imx27_imx25,
       .wait_op_done = wait_op_done_imx2x,
       .enable_hwecc = enable_hwecc_imx2x,
       .enable_spare_only = enable_spare_only_imx2x,
       .get_hwecc_status = get_hwecc_status_imx25,
       .preset = preset_imx25,
       .setup_data_interface = setup_data_interface_imx25,
       .send_cmd = send_cmd_imx2x,
       .send_addr = send_addr_imx2x,
       .trig_data_copy = trig_data_copy_imx2x,
       .correct_data = correct_data_imx25_imx5x,
       .ooblayout = &ooblayout_ops_imx25_imx5x,
       .regs_offset = 0x1e00,
       .spare0_offset = 0x1000,
       .axi_offset = 0,
       .spare_len = 64,
       .eccbytes = 9,
       .eccsize = 0,
       .eccstrength = 512,
       .prepad = 7,
       .postpad = 39,
};

/* v3.2a: i.MX51*/
static const struct mxc_nand_devtype_data imx51_nand_devtype_data = {
       .get_int = get_int_imx5x,
       .irq_control = irq_control_imx5x,
       .wait_op_done = wait_op_done_imx5x,
       .enable_hwecc = enable_hwecc_imx5x,
       .enable_spare_only = enable_spare_only_imx5x,
       .get_hwecc_status = get_hwecc_status_imx5x,
       .preset = preset_imx5x,
       .setup_data_interface = NULL,
       .send_cmd = send_cmd_imx5x,
       .send_addr = send_addr_imx5x,
       .trig_data_copy = trig_data_copy_imx5x,
       .correct_data = correct_data_imx25_imx5x,
       .ooblayout = &ooblayout_ops_imx25_imx5x,
       .regs_offset = 0,
       .spare0_offset = 0x1000,
       .axi_offset = 0x1e00,
       .spare_len = 64,
       .eccbytes = 0,
       .eccsize = 0,
       .ppb_shift = 7,
       .eccstrength = 512,
       /* Not working for 4kB data + 218B spare area NAND chips */
       .prepad = 8,
       .postpad = 48,
};

/* v3.2b: i.MX53 */
static const struct mxc_nand_devtype_data imx53_nand_devtype_data = {
       .get_int = get_int_imx5x,
       .irq_control = irq_control_imx5x,
       .wait_op_done = wait_op_done_imx5x,
       .enable_hwecc = enable_hwecc_imx5x,
       .enable_spare_only = enable_spare_only_imx5x,
       .get_hwecc_status = get_hwecc_status_imx5x,
       .preset = preset_imx5x,
       .setup_data_interface = NULL,
       .send_cmd = send_cmd_imx5x,
       .send_addr = send_addr_imx5x,
       .trig_data_copy = trig_data_copy_imx5x,
       .correct_data = correct_data_imx25_imx5x,
       .ooblayout = &ooblayout_ops_imx25_imx5x,
       .regs_offset = 0,
       .spare0_offset = 0x1000,
       .axi_offset = 0x1e00,
       .spare_len = 64,
       .eccbytes = 0,
       .eccsize = 0,
       .ppb_shift = 8,
       .eccstrength = 512,
       /* Not working for 4kB data + 218B spare area NAND chips */
       .prepad = 8,
       .postpad = 48,
};

static inline bool is_imx21_nfc(struct mxc_nfc *nfc)
{
       return nfc->devtype_data == &imx21_nand_devtype_data;
}

static inline bool is_imx27_nfc(struct mxc_nfc *nfc)
{
       return nfc->devtype_data == &imx27_nand_devtype_data;
}

static inline bool is_imx25_nfc(struct mxc_nfc *nfc)
{
       return nfc->devtype_data == &imx25_nand_devtype_data;
}

static inline bool is_imx51_nfc(struct mxc_nfc *nfc)
{
       return nfc->devtype_data == &imx51_nand_devtype_data;
}

static inline bool is_imx53_nfc(struct mxc_nfc *nfc)
{
       return nfc->devtype_data == &imx53_nand_devtype_data;
}

static struct platform_device_id mxc_nfc_devtype[] = {
       {
               .name = "imx21-nand",
               .driver_data = (kernel_ulong_t) &imx21_nand_devtype_data,
       }, {
               .name = "imx27-nand",
               .driver_data = (kernel_ulong_t) &imx27_nand_devtype_data,
       }, {
               .name = "imx25-nand",
               .driver_data = (kernel_ulong_t) &imx25_nand_devtype_data,
       }, {
               .name = "imx51-nand",
               .driver_data = (kernel_ulong_t) &imx51_nand_devtype_data,
       }, {
               .name = "imx53-nand",
               .driver_data = (kernel_ulong_t) &imx53_nand_devtype_data,
       }, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, mxc_nfc_devtype);

static const struct of_device_id mxc_nfc_ids[] = {
       {
               .compatible = "fsl,imx21-nand",
               .data = &imx21_nand_devtype_data,
       }, {
               .compatible = "fsl,imx27-nand",
               .data = &imx27_nand_devtype_data,
       }, {
               .compatible = "fsl,imx25-nand",
               .data = &imx25_nand_devtype_data,
       }, {
               .compatible = "fsl,imx51-nand",
               .data = &imx51_nand_devtype_data,
       }, {
               .compatible = "fsl,imx53-nand",
               .data = &imx53_nand_devtype_data,
       },
       { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_nfc_ids);

static int mxc_nand_chip_init(struct device *dev, struct mxc_nfc *nfc, int id)
{
       struct nand_chip *nand = &nfc->chips[id].nand;
       struct mtd_info *mtd;
       struct mtd_oob_region oobregion = { };
       int err;

       /* 50 us command delay time */
       nand->chip_delay = 50;
       nand->controller = &nfc->controller;
       nand_set_flash_node(nand, dev->of_node);
       nand->cmd_ctrl = mxc_nfc_cmd_ctrl;
       nand->read_byte = mxc_nfc_read_byte;
       nand->read_word = mxc_nfc_read_word;
       nand->read_buf = mxc_nfc_read_buf;
       nand->write_buf = mxc_nfc_write_buf;
       nand->select_chip = mxc_nfc_select_chip;
       nand->ecc.bytes = nfc->devtype_data->eccbytes;
       nand->ecc.size = nfc->devtype_data->eccstrength;
       nand->setup_data_interface = nfc->devtype_data->setup_data_interface;

       mtd = nand_to_mtd(nand);
       mtd->dev.parent = dev;
       mtd->name = DRIVER_NAME;

       /* First scan to find the device and get the page size */
       err = nand_scan_ident(mtd, is_imx25_nfc(nfc) ? 4 : 1, NULL);
       if (err)
               goto disable_clk;

       switch (nand->ecc.mode) {
       case NAND_ECC_NONE:
               break;
       case NAND_ECC_SOFT:
               if (nand->ecc.algo != NAND_ECC_BCH) {
                       err = -EINVAL;
                       goto disable_clk;
               }
               dev_dbg(dev, "using software BCH ECC engine\n");
               if (!nand->ecc.strength)
                       nand->ecc.strength = nand->ecc_strength_ds;

               if (!nand->ecc.size)
                       nand->ecc.size = nand->ecc_step_ds;
               mtd_set_ooblayout(mtd, NULL);
               break;
       case NAND_ECC_HW_SYNDROME:
               dev_dbg(dev, "using hardware syndrome ECC engine\n");
               /*
                * i.MX27 NFC: it happens for 3bit+ errors to read a status
                * showing "one bitflip has been corrected" (no uncorrectable
                * bitflip): nand is obviously wrong !
                */
               if (is_imx27_nfc(nfc))
                       dev_warn(dev, "HWECC not reliable for 3bit+ errors\n");
               nand->ecc.mode = NAND_ECC_HW_SYNDROME;
               nand->ecc.read_page = mxc_nfc_read_page_hwecc;
               nand->ecc.read_oob = mxc_nfc_read_oob_hwecc;
               nand->ecc.read_oob_raw = mxc_nfc_read_oob_raw_hwecc;
               nand->ecc.write_page = mxc_nfc_write_page_hwecc;
               nand->ecc.strength = get_eccsize(mtd);
               nand->ecc.steps = mtd->writesize / nand->ecc.size;
               nfc->devtype_data->ooblayout->ecc(mtd, 0, &oobregion);
               nand->ecc.prepad = nfc->devtype_data->prepad;
               nand->ecc.postpad = nfc->devtype_data->postpad;
               mtd_set_ooblayout(mtd, nfc->devtype_data->ooblayout);
               break;
       case NAND_ECC_HW:
       default:
               err = -EINVAL;
               goto disable_clk;
       }

       /* Call preset, with correct writesize */
       nfc->devtype_data->preset(mtd);

       if (nand->bbt_options & NAND_BBT_USE_FLASH) {
               nand->bbt_td = &bbt_main_descr;
               nand->bbt_md = &bbt_mirror_descr;
       }

       /* Second phase scan */
       err = nand_scan_tail(mtd);
       if (err) {
               dev_err(dev, "nand_scan_tail failed: %d\n", err);
               goto disable_clk;
       }

       /* Register the partitions */
       err = mtd_device_parse_register(mtd, part_probes, NULL,
                                       to_mxc_chip(nand)->pdata.parts,
                                       to_mxc_chip(nand)->pdata.nr_parts);
       if (err) {
               dev_err(dev, "failed to register mtd device: %d\n", err);
               nand_release(mtd);
               goto disable_clk;
       }

       return 0;

disable_clk:
       if (nfc->clk_act)
               clk_disable_unprepare(nfc->clk);

       return err;
}

static int mxc_nand_chips_init(struct device *dev, struct mxc_nfc *nfc)
{
       int ret = 0, id;

       for (id = 0; id < MXC_MAX_NAND_CHIPS; id++) {
               ret = mxc_nand_chip_init(dev, nfc, id);
               if (ret)
                       break;
       }

       return ret;
}

static int mxc_nfc_probe(struct platform_device *pdev)
{
       struct device *dev = &pdev->dev;
       struct resource *res;
       struct mxc_nfc *nfc;
       const struct of_device_id *of_id = of_match_device(mxc_nfc_ids, dev);
       int err = 0;

       /* Allocate memory for NFC and NAND chips structures */
       nfc = devm_kzalloc(dev, sizeof(struct mxc_nfc),
                       GFP_KERNEL);
       if (!nfc)
               return -ENOMEM;

       /* Structures must be linked */
       nfc->dev = dev;
       nand_hw_control_init(&nfc->controller);
       nfc->devtype_data = of_id->data;

       nfc->clk = devm_clk_get(dev, NULL);
       if (IS_ERR(nfc->clk))
               return PTR_ERR(nfc->clk);

       res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
       nfc->base = devm_ioremap_resource(dev, res);
       if (IS_ERR(nfc->base))
               return PTR_ERR(nfc->base);

       if (is_imx51_nfc(nfc) || is_imx53_nfc(nfc)) {
               res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
               nfc->regs_ip = devm_ioremap_resource(&pdev->dev, res);
               if (IS_ERR(nfc->regs_ip))
                       return PTR_ERR(nfc->regs_ip);
       }

       nfc->main_area0 = nfc->base;
       nfc->spare0 = nfc->base + nfc->devtype_data->spare0_offset;
       nfc->regs = nfc->base + nfc->devtype_data->regs_offset;
       nfc->regs_axi = nfc->base + nfc->devtype_data->axi_offset;

       /* Allocate the subpage-sized buffer */
       nfc->data_buf = devm_kzalloc(dev, nfc->devtype_data->eccstrength +
                                    nfc->devtype_data->spare_len,
                                    GFP_KERNEL);
       if (!nfc->data_buf)
               return -ENOMEM;

       init_completion(&nfc->op_completion);

       nfc->irq = platform_get_irq(pdev, 0);
       if (nfc->irq < 0)
               return nfc->irq;

       nfc->devtype_data->irq_control(nfc, 0);

       err = devm_request_irq(dev, nfc->irq, nfc_irq,
                       0, DRIVER_NAME, nfc);
       if (err)
               return err;

       err = clk_prepare_enable(nfc->clk);
       if (err)
               return err;
       nfc->clk_act = 1;

       /*
        * Now that we "own" the interrupt make sure the interrupt mask bit is
        * cleared on i.MX21. Otherwise we can't read the interrupt status bit
        * on this machine.
        */
       if (is_imx21_nfc(nfc)) {
               disable_irq_nosync(nfc->irq);
               nfc->devtype_data->irq_control(nfc, 1);
       }

       platform_set_drvdata(pdev, nfc);

       /* Unlock the NFC SRAM Buffer before the first scan */
       if (is_imx21_nfc(nfc) || is_imx27_nfc(nfc) || is_imx25_nfc(nfc))
               writew(0x2, NFC_V1_V2_CONFIG);
       else
               writel(NFC_V3_WRPROT_BLS_UNLOCK | NFC_V3_WRPROT_UNLOCK,
                      NFC_V3_WRPROT);

       err = mxc_nand_chips_init(dev, nfc);
       if (err) {
               dev_err(dev, "failed to init nand chips\n");
               goto dis_clk;
       }

       return 0;

dis_clk:
       if (nfc->clk_act)
               clk_disable_unprepare(nfc->clk);

       return err;
}

static void mxc_nand_chips_cleanup(struct mxc_nfc *nfc)
{
       struct nand_chip *nand;
       int id;

       for (id = 0; id < MXC_MAX_NAND_CHIPS; id++) {
               nand = &nfc->chips[id].nand;
               nand_release(nand_to_mtd(nand));
       }
}

static int mxc_nfc_remove(struct platform_device *pdev)
{
       struct mxc_nfc *nfc = platform_get_drvdata(pdev);

       mxc_nand_chips_cleanup(nfc);

       clk_disable_unprepare(nfc->clk);

       return 0;
}

static struct platform_driver mxc_nfc_driver = {
       .driver = {
                  .name = DRIVER_NAME,
                  .of_match_table = of_match_ptr(mxc_nfc_ids),
       },
       .id_table = mxc_nfc_devtype,
       .probe = mxc_nfc_probe,
       .remove = mxc_nfc_remove,
};
module_platform_driver(mxc_nfc_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX27 NAND flash controller driver");
MODULE_LICENSE("GPL");
