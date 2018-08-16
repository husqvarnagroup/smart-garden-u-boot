// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Stefan Roese <sr@denx.de>
 *
 * MediaTek ethernet IP driver for U-Boot
 * test-only: add based on MediaTek code ....
 */

#include <common.h>
#include <dm.h>
#include <hexdump.h> // test-only
#include <linux/err.h>
#include <malloc.h>
#include <miiphy.h>
#include <net.h>
#include <asm/io.h>


#if 0 // test-only
// aaa
#undef printf
#define printf
#endif

#if 0 // test-only
#undef print_hex_dump_bytes
#define print_hex_dump_bytes
#endif





#define RALINK_SYSCTL_BASE        0xb0000000
#define RALINK_ETH_SW_BASE              0xB0110000
#define RALINK_FRAME_ENGINE_BASE        0xB0100000

#define RT2880_SYS_CNTL_BASE			(RALINK_SYSCTL_BASE)

#define RT2880_AGPIOCFG_REG			(RT2880_SYS_CNTL_BASE+0x3c)

#define RT2880_RSTCTRL_REG			(RT2880_SYS_CNTL_BASE+0x34)


// test-only!!!
#define NUM_RX_DESC 256
#define NUM_TX_DESC 16

#undef PKTBUFSRX
#define PKTBUFSRX	NUM_RX_DESC

#define PADDING_LENGTH		60

#define CFG_HZ			100000 // test-only


#define RALINK_REG(x)		(*((volatile u32 *)(x)))

#define ra_inb(offset)		(*(volatile unsigned char *)(offset))
#define ra_inw(offset)		(*(volatile unsigned short *)(offset))
#define ra_inl(offset)		(*(volatile unsigned long *)(offset))

#define ra_outb(offset,val)	(*(volatile unsigned char *)(offset) = val)
#define ra_outw(offset,val)	(*(volatile unsigned short *)(offset) = val)
#define ra_outl(offset,val)	(*(volatile unsigned long *)(offset) = val)

#define ra_and(addr, value) ra_outl(addr, (ra_inl(addr) & (value)))
#define ra_or(addr, value) ra_outl(addr, (ra_inl(addr) | (value)))



#define outw(address, value)    *((volatile uint32_t *)(address)) = cpu_to_le32(value)
#define inw(address)            le32_to_cpu(*(volatile u32 *)(address))

#define PHY_CONTROL_0 		0xC0
#define PHY_CONTROL_1 		0xC4
#define MDIO_PHY_CONTROL_0	(RALINK_ETH_SW_BASE + PHY_CONTROL_0)
#define MDIO_PHY_CONTROL_1 	(RALINK_ETH_SW_BASE + PHY_CONTROL_1)




/* ====================================== */
//GDMA1 uni-cast frames destination port
#define GDM_UFRC_P_CPU     ((u32)(~(0x7 << 12)))
#define GDM_UFRC_P_GDMA1   (1 << 12)
#define GDM_UFRC_P_GDMA2   (2 << 12)
#define GDM_UFRC_P_DROP    (7 << 12)
//GDMA1 broad-cast MAC address frames
#define GDM_BFRC_P_CPU     ((u32)(~(0x7 << 8)))
#define GDM_BFRC_P_GDMA1   (1 << 8)
#define GDM_BFRC_P_GDMA2   (2 << 8)
#define GDM_BFRC_P_PPE     (6 << 8)
#define GDM_BFRC_P_DROP    (7 << 8)
//GDMA1 multi-cast MAC address frames
#define GDM_MFRC_P_CPU     ((u32)(~(0x7 << 4)))
#define GDM_MFRC_P_GDMA1   (1 << 4)
#define GDM_MFRC_P_GDMA2   (2 << 4)
#define GDM_MFRC_P_PPE     (6 << 4)
#define GDM_MFRC_P_DROP    (7 << 4)
//GDMA1 other MAC address frames destination port
#define GDM_OFRC_P_CPU     ((u32)(~(0x7)))
#define GDM_OFRC_P_GDMA1   1
#define GDM_OFRC_P_GDMA2   2
#define GDM_OFRC_P_PPE     6
#define GDM_OFRC_P_DROP    7

#define RST_DRX_IDX0      BIT(16)
#define RST_DTX_IDX0      BIT(0)

#define TX_WB_DDONE       BIT(6)
#define RX_DMA_BUSY       BIT(3)
#define TX_DMA_BUSY       BIT(1)
#define RX_DMA_EN         BIT(2)
#define TX_DMA_EN         BIT(0)

#define GP1_FRC_EN        BIT(15)
#define GP1_FC_TX         BIT(11)
#define GP1_FC_RX         BIT(10)
#define GP1_LNK_DWN       BIT(9)
#define GP1_AN_OK         BIT(8)

/*
 * FE_INT_STATUS
 */
#define CNT_PPE_AF       BIT(31)
#define CNT_GDM1_AF      BIT(29)
#define PSE_P1_FC        BIT(22)
#define PSE_P0_FC        BIT(21)
#define PSE_FQ_EMPTY     BIT(20)
#define GE1_STA_CHG      BIT(18)
#define TX_COHERENT      BIT(17)
#define RX_COHERENT      BIT(16)

#define TX_DONE_INT1     BIT(9)
#define TX_DONE_INT0     BIT(8)
#define RX_DONE_INT0     BIT(2)
#define TX_DLY_INT       BIT(1)
#define RX_DLY_INT       BIT(0)

/*
 * Ethernet chip registers.RT2880
 */

#define PDMA_RELATED		0x0800
/* 1. PDMA */
#define TX_BASE_PTR0            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x000)
#define TX_MAX_CNT0             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x004)
#define TX_CTX_IDX0             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x008)
#define TX_DTX_IDX0             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x00C)

#define TX_BASE_PTR1            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x010)
#define TX_MAX_CNT1             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x014)
#define TX_CTX_IDX1             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x018)
#define TX_DTX_IDX1             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x01C)

#define TX_BASE_PTR2            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x020)
#define TX_MAX_CNT2             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x024)
#define TX_CTX_IDX2             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x028)
#define TX_DTX_IDX2             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x02C)

#define TX_BASE_PTR3            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x030)
#define TX_MAX_CNT3             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x034)
#define TX_CTX_IDX3             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x038)
#define TX_DTX_IDX3             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x03C)

#define RX_BASE_PTR0            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x100)
#define RX_MAX_CNT0             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x104)
#define RX_CALC_IDX0            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x108)
#define RX_DRX_IDX0             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x10C)

#define RX_BASE_PTR1            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x110)
#define RX_MAX_CNT1             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x114)
#define RX_CALC_IDX1            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x118)
#define RX_DRX_IDX1             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x11C)

#define PDMA_INFO               (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x200)
#define PDMA_GLO_CFG            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x204)
#define PDMA_RST_IDX            (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x208)
#define PDMA_RST_CFG            (RALINK_FRAME_ENGINE_BASE + PDMA_RST_IDX)
#define DLY_INT_CFG             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x20C)
#define FREEQ_THRES             (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x210)
#define INT_STATUS              (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x220)
#define FE_INT_STATUS           (INT_STATUS)
#define INT_MASK                (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x228)
#define FE_INT_ENABLE           (INT_MASK)
#define PDMA_WRR                (RALINK_FRAME_ENGINE_BASE + PDMA_RELATED+0x280)
#define PDMA_SCH_CFG            (PDMA_WRR)

#define SDM_RELATED		0x0C00
#define SDM_CON                 (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x00)  //Switch DMA configuration
#define SDM_RRING               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x04)  //Switch DMA Rx Ring
#define SDM_TRING               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x08)  //Switch DMA Tx Ring
#define SDM_MAC_ADRL            (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x0C)  //Switch MAC address LSB
#define SDM_MAC_ADRH            (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x10)  //Switch MAC Address MSB
#define SDM_TPCNT               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x100) //Switch DMA Tx packet count
#define SDM_TBCNT               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x104) //Switch DMA Tx byte count
#define SDM_RPCNT               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x108) //Switch DMA rx packet count
#define SDM_RBCNT               (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x10C) //Switch DMA rx byte count
#define SDM_CS_ERR              (RALINK_FRAME_ENGINE_BASE + SDM_RELATED+0x110) //Switch DMA rx checksum error count

// test-only: don't use bit masks here but normal structs (Linux driver)
#if 0
/*=========================================
      PDMA RX Descriptor Format define
=========================================*/

//-------------------------------------------------
typedef struct _PDMA_RXD_INFO1_  PDMA_RXD_INFO1_T;

struct _PDMA_RXD_INFO1_
{
    unsigned int    PDP0;
};


//-------------------------------------------------
typedef struct _PDMA_RXD_INFO2_    PDMA_RXD_INFO2_T;

struct _PDMA_RXD_INFO2_
{
	unsigned int    PLEN1                   : 14;
	unsigned int    LS1                     : 1;
	unsigned int    UN_USED                 : 1;
	unsigned int    PLEN0                   : 14;
	unsigned int    LS0                     : 1;
	unsigned int    DDONE_bit               : 1;
};
//-------------------------------------------------
typedef struct _PDMA_RXD_INFO3_  PDMA_RXD_INFO3_T;

struct _PDMA_RXD_INFO3_
{
	unsigned int    PDP1;
};
//-------------------------------------------------
typedef struct _PDMA_RXD_INFO4_    PDMA_RXD_INFO4_T;

struct _PDMA_RXD_INFO4_
{
	unsigned int    FOE_Entry               : 14;
	unsigned int    FVLD                    : 1;
	unsigned int    UN_USE1                 : 1;
	unsigned int    AI                      : 8;
	unsigned int    SP                      : 3;
	unsigned int    AIS                     : 1;
	unsigned int    L4F                     : 1;
	unsigned int    IPF                     : 1;
	unsigned int    L4FVLD_bit              : 1;
	unsigned int    IPFVLD_bit              : 1;
};

struct PDMA_rxdesc {
	PDMA_RXD_INFO1_T rxd_info1;
	PDMA_RXD_INFO2_T rxd_info2;
	PDMA_RXD_INFO3_T rxd_info3;
	PDMA_RXD_INFO4_T rxd_info4;
};
/*=========================================
      PDMA TX Descriptor Format define
=========================================*/
//-------------------------------------------------
typedef struct _PDMA_TXD_INFO1_  PDMA_TXD_INFO1_T;

struct _PDMA_TXD_INFO1_
{
	unsigned int    SDP0;
};
//-------------------------------------------------
typedef struct _PDMA_TXD_INFO2_    PDMA_TXD_INFO2_T;

struct _PDMA_TXD_INFO2_
{
	unsigned int    SDL1                  : 14;
	unsigned int    LS1_bit               : 1;
	unsigned int    BURST_bit             : 1;
	unsigned int    SDL0                  : 14;
	unsigned int    LS0_bit               : 1;
	unsigned int    DDONE_bit             : 1;
};
//-------------------------------------------------
typedef struct _PDMA_TXD_INFO3_  PDMA_TXD_INFO3_T;

struct _PDMA_TXD_INFO3_
{
	unsigned int    SDP1;
};
//-------------------------------------------------
typedef struct _PDMA_TXD_INFO4_    PDMA_TXD_INFO4_T;

struct _PDMA_TXD_INFO4_
{
    unsigned int    VIDX		: 4;
    unsigned int    VPRI                : 3;
    unsigned int    INSV                : 1;
    unsigned int    SIDX                : 4;
    unsigned int    INSP                : 1;
    unsigned int    UN_USE3             : 3;
    unsigned int    QN                  : 3;
    unsigned int    UN_USE2             : 5;
    unsigned int    PN                  : 3;
    unsigned int    UN_USE1             : 2;
    unsigned int    TUI_CO              : 3;
};

struct PDMA_txdesc {
	PDMA_TXD_INFO1_T txd_info1;
	PDMA_TXD_INFO2_T txd_info2;
	PDMA_TXD_INFO3_T txd_info3;
	PDMA_TXD_INFO4_T txd_info4;
};

#else

/* rxd2 */
#define RX_DMA_DONE		BIT(31)
#define RX_DMA_LSO		BIT(30)
#define RX_DMA_PLEN0(_x)	(((_x) & 0x3fff) << 16)
#define RX_DMA_GET_PLEN0(_x)	(((_x) >> 16) & 0x3fff)
#define RX_DMA_TAG		BIT(15)
/* rxd3 */
#define RX_DMA_TPID(_x)		(((_x) >> 16) & 0xffff)
#define RX_DMA_VID(_x)		((_x) & 0xffff)
/* rxd4 */
#define RX_DMA_L4VALID		BIT(30)

struct fe_rx_dma {
	unsigned int rxd1;
	unsigned int rxd2;
	unsigned int rxd3;
	unsigned int rxd4;
} __packed __aligned(4);

#define TX_DMA_BUF_LEN		0x3fff
#define TX_DMA_PLEN0_MASK	(TX_DMA_BUF_LEN << 16)
#define TX_DMA_PLEN0(_x)	(((_x) & TX_DMA_BUF_LEN) << 16)
#define TX_DMA_PLEN1(_x)	((_x) & TX_DMA_BUF_LEN)
#define TX_DMA_GET_PLEN0(_x)    (((_x) >> 16) & TX_DMA_BUF_LEN)
#define TX_DMA_GET_PLEN1(_x)    ((_x) & TX_DMA_BUF_LEN)
#define TX_DMA_LS1		BIT(14)
#define TX_DMA_LS0		BIT(30)
#define TX_DMA_DONE		BIT(31)

#define TX_DMA_INS_VLAN_MT7621	BIT(16)
#define TX_DMA_INS_VLAN		BIT(7)
#define TX_DMA_INS_PPPOE	BIT(12)
#define TX_DMA_QN(_x)		((_x) << 16)
#define TX_DMA_PN(_x)		((_x) << 24)
#define TX_DMA_QN_MASK		TX_DMA_QN(0x7)
#define TX_DMA_PN_MASK		TX_DMA_PN(0x7)
#define TX_DMA_UDF		BIT(20)
#define TX_DMA_CHKSUM		(0x7 << 29)
#define TX_DMA_TSO		BIT(28)

struct fe_tx_dma {
	unsigned int txd1;
	unsigned int txd2;
	unsigned int txd3;
	unsigned int txd4;
} __packed __aligned(4);

#endif


static int rx_dma_owner_idx0;	/* Point to the next RXD DMA wants to use in RXD Ring#0.  */
static int rx_wants_alloc_idx0;	/* Point to the next RXD CPU wants to allocate to RXD Ring #0. */
static int tx_cpu_owner_idx0;	/* Point to the next TXD in TXD_Ring0 CPU wants to use */

struct mt76xx_eth_dev {
//	struct emac_regs *regs;
	struct mii_dev *bus;
	struct phy_device *phydev;
	int link_printed;

//	u8 rx_buf[EMAC_RX_BUFSIZE];
	// test-only: this cache stuff needed???
#if 0
	struct PDMA_txdesc *tx_ring;
	struct PDMA_rxdesc *rx_ring;
#else
	struct fe_tx_dma *tx_ring;
	struct fe_rx_dma *rx_ring;
#endif

	u8 *rx_buf[NUM_RX_DESC];
};


#define MTK_QDMA_PAGE_SIZE	2048

// test-only: CFG_HZ ???
#define CONFIG_MDIO_TIMEOUT	100

static int mdio_wait_read(u32 mask, bool mask_set)
{
	int timeout = CONFIG_MDIO_TIMEOUT;
	int start;

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (mask_set) {
			if (inw(MDIO_PHY_CONTROL_1) & mask)
				return 0;
		} else {
			if (!(inw(MDIO_PHY_CONTROL_1) & mask))
				return 0;
		}
	}

	printf("MDIO operation timeout!\n");

	return -ETIMEDOUT;
}

static int mii_mgr_read(u32 phy_addr, u32 phy_register, u32 *read_data)
{
	u32 status = 0;
	u32 ret;

	*read_data = 0xffff;
	// test-only: what does this comment mean ???
	/* We enable mdio gpio purpose register, and disable it when exit */
	// make sure previous read operation is complete
	ret = mdio_wait_read(BIT(1), false);
	if (ret)
		return ret;

	outw(MDIO_PHY_CONTROL_0 , BIT(14) | (phy_register << 8) | phy_addr);

	// make sure read operation is complete
	ret = mdio_wait_read(BIT(1), true);
	if (ret)
		return ret;

	status = inw(MDIO_PHY_CONTROL_1);
	*read_data = (u32)(status >> 16);
	return 0;
}

static int mii_mgr_write(u32 phy_addr, u32 phy_register, u32 write_data)
{
	u32 data;
	int ret;

	// make sure previous write operation is complete
	ret = mdio_wait_read(BIT(0), false);
	if (ret)
		return ret;

	data = (write_data & 0xffff) << 16;
	data |= (phy_register << 8) | phy_addr;
	data |= BIT(13);
	outw(MDIO_PHY_CONTROL_0, data);

	return mdio_wait_read(BIT(0), true);
}

static int mt76xx_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	u32 val;
	int ret;

	ret = mii_mgr_read(addr, reg, &val);
	if (ret)
		return ret;

	return val;
}

static int mt76xx_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			     u16 value)
{
	return mii_mgr_write(addr, reg, value);
}

static void mt7628_ephy_init(void)
{
	int i;

	mii_mgr_write(0, 31, 0x2000);	/* change G2 page */
	mii_mgr_write(0, 26, 0x0000);

	for (i = 0; i < 5; i++) {
		mii_mgr_write(i, 31, 0x8000);	/*change L0 page */
		mii_mgr_write(i,  0, 0x3100);

		/* EEE disable */
		mii_mgr_write(i, 30, 0xa000);
		mii_mgr_write(i, 31, 0xa000);	/* change L2 page */
		mii_mgr_write(i, 16, 0x0606);
		mii_mgr_write(i, 23, 0x0f0e);
		mii_mgr_write(i, 24, 0x1610);
		mii_mgr_write(i, 30, 0x1f15);
		mii_mgr_write(i, 28, 0x6111);
	}

        /* 100Base AOI setting */
	mii_mgr_write(0, 31, 0x5000);	/* change G5 page */
	mii_mgr_write(0, 19, 0x004a);
	mii_mgr_write(0, 20, 0x015a);
	mii_mgr_write(0, 21, 0x00ee);
	mii_mgr_write(0, 22, 0x0033);
	mii_mgr_write(0, 23, 0x020a);
	mii_mgr_write(0, 24, 0x0000);
	mii_mgr_write(0, 25, 0x024a);
	mii_mgr_write(0, 26, 0x035a);
	mii_mgr_write(0, 27, 0x02ee);
	mii_mgr_write(0, 28, 0x0233);
	mii_mgr_write(0, 29, 0x000a);
	mii_mgr_write(0, 30, 0x0000);

	/* Fix EPHY idle state abnormal behavior */
	mii_mgr_write(0, 31, 0x4000);	/* change G4 page */
	mii_mgr_write(0, 29, 0x000d);
	mii_mgr_write(0, 30, 0x0500);
}

static void rt305x_esw_init(void)
{
	u32 i;

	/*
	 * FC_RLS_TH=200, FC_SET_TH=160
	 * DROP_RLS=120, DROP_SET_TH=80
	 */
	RALINK_REG(RALINK_ETH_SW_BASE+0x0008) = 0xc8a07850;
	RALINK_REG(RALINK_ETH_SW_BASE+0x00E4) = 0x00000000;
	RALINK_REG(RALINK_ETH_SW_BASE+0x0014) = 0x00405555;
	RALINK_REG(RALINK_ETH_SW_BASE+0x0090) = 0x00007f7f;
	RALINK_REG(RALINK_ETH_SW_BASE+0x0098) = 0x00007f7f; //disable VLAN
	RALINK_REG(RALINK_ETH_SW_BASE+0x00CC) = 0x0002500c;
	RALINK_REG(RALINK_ETH_SW_BASE+0x009C) = 0x0008a301; //hashing algorithm=XOR48, aging interval=300sec
	RALINK_REG(RALINK_ETH_SW_BASE+0x008C) = 0x02404040;

	RALINK_REG(RALINK_ETH_SW_BASE+0x00C8) = 0x3f502b28; //Ext PHY Addr=0x1f
	RALINK_REG(RALINK_ETH_SW_BASE+0x0084) = 0x00000000;
	RALINK_REG(RALINK_ETH_SW_BASE+0x0110) = 0x7d000000; //1us cycle number=125 (FE's clock=125Mhz)




#define RSTCTRL_EPHY_RST	BIT(24)
#define MT7628_EPHY_EN	        (0x1f << 16)
#define MT7628_P0_EPHY_AIO_EN   BIT(16)
	/* We shall prevent modifying PHY registers if it is FPGA mode */
	i = RALINK_REG(RT2880_AGPIOCFG_REG);
        i |= MT7628_EPHY_EN;
        i = i & ~(MT7628_P0_EPHY_AIO_EN);
	RALINK_REG(RT2880_AGPIOCFG_REG) = i;

	debug("Reset MT7628 PHY!\n");
	/* Reset PHY */
	i = RALINK_REG(RT2880_RSTCTRL_REG);
	i = i | RSTCTRL_EPHY_RST;
	RALINK_REG(RT2880_RSTCTRL_REG) = i;
	i = i & ~(RSTCTRL_EPHY_RST);
	RALINK_REG(RT2880_RSTCTRL_REG) = i;
	i = RALINK_REG(RALINK_SYSCTL_BASE + 0x64);
        i &= 0xf003f003;
        i |= 0x05540554;
        RALINK_REG(RALINK_SYSCTL_BASE + 0x64) = i; /* set P0 EPHY LED mode */

	udelay(5000);
	mt7628_ephy_init();
}

// test-only: needed???
//static void START_ETH(void)
static void eth_dma_start(void)
{
	u32 val;

	wmb();
	val = RALINK_REG(PDMA_GLO_CFG);
	val |= TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN;
	RALINK_REG(PDMA_GLO_CFG) = val;
}

//static void STOP_ETH(void)
static void eth_dma_stop(void)
{
#define CONFIG_DMA_STOP_TIMEOUT		100
	int timeout = CONFIG_DMA_STOP_TIMEOUT;
	int start;
	u32 val;

	wmb();
	val = RALINK_REG(PDMA_GLO_CFG);
	val &= ~(TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN);
	RALINK_REG(PDMA_GLO_CFG) = val;

	/* Wait for DMA to stop */
	start = get_timer(0);
	while (get_timer(start) < timeout) {
		val = RALINK_REG(PDMA_GLO_CFG);
		if ((val & (RX_DMA_BUSY | TX_DMA_BUSY)) == 0)
			return;
	}

	printf("DMA stop timeout error!\n"); // test-only
}

#if 0
//static int isDMABusy(struct udevice *dev)
static bool eth_dma_busy(struct udevice *dev)
{
	u32 reg;

	reg = RALINK_REG(PDMA_GLO_CFG);

	// test-only: move to debug() ???
	if (reg & RX_DMA_BUSY) {
		printf("RX_DMA_BUSY !!!\n");
		return true;
	}

	if (reg & TX_DMA_BUSY) {
		printf("TX_DMA_BUSY !!!\n");
		return true;
	}

	return false;
}
#endif

static int mt76xx_eth_write_hwaddr(struct udevice *dev)
{
	u8 *addr = ((struct eth_pdata *)dev_get_platdata(dev))->enetaddr;
	u32 val;
	u16 tmp;

	/* Set MAC address. */
	tmp = (u16)addr[0];
	val = (tmp << 8) | addr[1];

	RALINK_REG(SDM_MAC_ADRH) = val;
	// printf("\n dev->iobase=%08X,SDM_MAC_ADRH=%08X\n",dev->iobase,val);

	tmp = (u16)addr[2];
	val = (tmp << 8) | addr[3];
	val = val << 16;
	tmp = (u16)addr[4];
	val |= (tmp<<8) | addr[5];
	RALINK_REG(SDM_MAC_ADRL) = val;

#if 0
	printf("rt2880_eth_init, set MAC reg to [%02X:%02X:%02X:%02X:%02X:%02X]\n",
	       addr[0], addr[1], addr[2],
	       addr[3], addr[4], addr[5]);
#endif

	return 0;
}

static int mt76xx_eth_recv(struct udevice *dev, int flags, uchar **packetp); // test-only

static int mt76xx_eth_start(struct udevice *dev)
{
	struct mt76xx_eth_dev *priv = dev_get_priv(dev);
	u32 val;
	int i;

	for (i = 0; i < NUM_RX_DESC; i++) {
		memset((void *)&priv->rx_ring[i], 0, sizeof(priv->rx_ring[0]));
#if 0
		priv->rx_ring[i].rxd_info2.DDONE_bit = 0;
		priv->rx_ring[i].rxd_info2.LS0 = 1;
		priv->rx_ring[i].rxd_info1.PDP0 = CPHYSADDR(priv->rx_buf[i]);
#else
		priv->rx_ring[i].rxd2 |= RX_DMA_LSO;
		priv->rx_ring[i].rxd1 = CPHYSADDR(priv->rx_buf[i]);
#endif
	}

	for (i = 0; i < NUM_TX_DESC; i++) {
		memset((void *)&priv->tx_ring[i], 0, sizeof(priv->tx_ring[0]));
#if 0
		priv->tx_ring[i].txd_info2.LS0_bit = 1;
		priv->tx_ring[i].txd_info2.DDONE_bit = 1;
		/* PN:
		 *  0:CPU
		 *  1:GE1
		 *  2:GE2 (for RT2883)
		 *  6:PPE
		 *  7:Discard
		 */
		priv->tx_ring[i].txd_info4.PN = 1;
#else
		priv->tx_ring[i].txd2 = TX_DMA_LS0 | TX_DMA_DONE;
		priv->tx_ring[i].txd4 = TX_DMA_PN(1);
#endif
	}

	rx_dma_owner_idx0 = 0;
	rx_wants_alloc_idx0 = (NUM_RX_DESC - 1);
	tx_cpu_owner_idx0 = 0;

	wmb(); // test-only

	/* disable delay interrupt */
	RALINK_REG(DLY_INT_CFG) = 0;

	val = RALINK_REG(PDMA_GLO_CFG);
	udelay(100);

	{
		val &= 0x0000FFFF;

		RALINK_REG(PDMA_GLO_CFG)=val;
		udelay(500);
		val=RALINK_REG(PDMA_GLO_CFG);
	}

	wmb(); // test-only

	/* Tell the adapter where the TX/RX rings are located. */
	RALINK_REG(RX_BASE_PTR0) = CPHYSADDR(&priv->rx_ring[0]);

	//printf("\n rx_ring=%08X ,RX_BASE_PTR0 = %08X \n",&rx_ring[0],RALINK_REG(RX_BASE_PTR0));
	RALINK_REG(TX_BASE_PTR0) = CPHYSADDR((u32)&priv->tx_ring[0]);

	//printf("\n tx_ring0=%08X, TX_BASE_PTR0 = %08X \n",&tx_ring0[0],RALINK_REG(TX_BASE_PTR0));

	RALINK_REG(RX_MAX_CNT0)=cpu_to_le32((u32)NUM_RX_DESC);
	RALINK_REG(TX_MAX_CNT0)=cpu_to_le32((u32)NUM_TX_DESC);

	RALINK_REG(TX_CTX_IDX0)=cpu_to_le32((u32)tx_cpu_owner_idx0);
	RALINK_REG(PDMA_RST_IDX)=cpu_to_le32((u32)RST_DTX_IDX0);

	RALINK_REG(RX_CALC_IDX0)=cpu_to_le32((u32)(NUM_RX_DESC - 1));
	RALINK_REG(PDMA_RST_IDX)=cpu_to_le32((u32)RST_DRX_IDX0);

//	printf("%s (%d): RX_CALC_IDX0=%d RX_DRX_IDX0=%d PDMA_INFO=%08x PDMA_GLO_CFG=%08x\n", __func__, __LINE__, RALINK_REG(RX_CALC_IDX0), RALINK_REG(RX_DRX_IDX0), RALINK_REG(PDMA_INFO), RALINK_REG(PDMA_GLO_CFG)); // test-only
//	udelay(500);
	wmb(); // test-only
	eth_dma_start();
//	printf("%s (%d): RX_CALC_IDX0=%d RX_DRX_IDX0=%d PDMA_INFO=%08x PDMA_GLO_CFG=%08x\n", __func__, __LINE__, RALINK_REG(RX_CALC_IDX0), RALINK_REG(RX_DRX_IDX0), RALINK_REG(PDMA_INFO), RALINK_REG(PDMA_GLO_CFG)); // test-only
#if 0 // test-only: this crashes in free in the hush shell
	uchar *packet;
	packet = malloc(2048);
	printf("packet=%p!!!!!!!!!!!!!!!!!\n", packet);
	mdelay(100);
	while (mt76xx_eth_recv(dev, 0, &packet) != -EAGAIN)
		;
//	invalidate_dcache_range((u32)packet, (u32)packet + 2048 - 1); // test-only
	free(packet);
#endif
#if 0 // test-only: this works somehow
	uchar packet[2048];
	mdelay(100);
	while (mt76xx_eth_recv(dev, 0, &packet) != -EAGAIN)
		;
#endif
#if 1 // test-only: this works somehow
	uchar packet[2048];
	uchar *packetp;

	mdelay(300);
	packetp = &packet[0];
	while (mt76xx_eth_recv(dev, 0, &packetp) != -EAGAIN)
		;
#endif
#if 0 // test-only: this works somehow
	mdelay(200);
#endif

	return 0;
}

// test-only
#define TOUT_LOOP   1000

static int mt76xx_eth_send(struct udevice *dev, void *packet, int length)
{
	struct mt76xx_eth_dev *priv = dev_get_priv(dev);
	char *p = (char *)packet; // test-only: needed??
//	int timeout = CONFIG_TX_DMA_TIMEOUT;
//	int start;
	u32 tmp;
	int i;

	/* Pad message to a minimum length */
	if (length < PADDING_LENGTH) {
		for (i = 0; i < PADDING_LENGTH - length; i++)
			p[length + i] = 0;
		length = PADDING_LENGTH;
	}

#if 0 // test-only: change to using no bitmasks and then wait_timeout_bit()
	start = get_timer(0);
	while (get_timer(start) < timeout) {
	}
#endif

#if 0
	for (i = 0; priv->tx_ring[tx_cpu_owner_idx0].txd_info2.DDONE_bit == 0;
	     i++) {
		if (i >= TOUT_LOOP) {
			printf("TX DMA is Busy !! TX desc is Empty!\n"); // test-only
//			goto Done;
			return 0; // test-only: return with error on timeout?
		}
	}
#endif

	tmp = RALINK_REG(TX_DTX_IDX0);

	if (tmp == (tx_cpu_owner_idx0 + 1) % NUM_TX_DESC) {
		puts(" @ ");
		printf("%s (%d)!!!!!!!!!!!!!!!!!!!!!!\n", __func__, __LINE__); // test-only
//		goto Done;
		return 0; // test-only: return with error on timeout?
	}

	flush_dcache_range((u32)packet, (u32)packet + length - 1); // test-only
#if 0
	priv->tx_ring[tx_cpu_owner_idx0].txd_info1.SDP0 = CPHYSADDR(packet);
	priv->tx_ring[tx_cpu_owner_idx0].txd_info2.SDL0 = length;
#else
	priv->tx_ring[tx_cpu_owner_idx0].txd1 = CPHYSADDR(packet);
	priv->tx_ring[tx_cpu_owner_idx0].txd2 |= TX_DMA_PLEN0(length);
#endif
//	print_hex_dump_bytes("TX: ", DUMP_PREFIX_OFFSET, packet, length); // test-only

#if 0
	priv->tx_ring[tx_cpu_owner_idx0].txd_info2.DDONE_bit = 0;
#else
	priv->tx_ring[tx_cpu_owner_idx0].txd2 &= ~TX_DMA_DONE;
#endif

	// test-only: do we need to increment here, if tx_ids is 1 ???
	tx_cpu_owner_idx0 = (tx_cpu_owner_idx0 + 1) % NUM_TX_DESC;

	wmb(); // test-only
	RALINK_REG(TX_CTX_IDX0) = cpu_to_le32((u32)tx_cpu_owner_idx0);

	return 0;
}

static int mt76xx_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
//	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct mt76xx_eth_dev *priv = dev_get_priv(dev);
	int length;
	u32 *rxd_info;

#if 0
	printf("%s (%d): rx_dma_owner_idx0=%d\n", __func__, __LINE__, rx_dma_owner_idx0); // test-only
	printf("%s (%d): RX_CALC_IDX0=%d RX_DRX_IDX0=%d PDMA_INFO=%08x PDMA_GLO_CFG=%08x\n", __func__, __LINE__, RALINK_REG(RX_CALC_IDX0), RALINK_REG(RX_DRX_IDX0), RALINK_REG(PDMA_INFO), RALINK_REG(PDMA_GLO_CFG)); // test-only
#endif

//	invalidate_dcache_range((u32)&priv->rx_ring[rx_dma_owner_idx0], (u32)&priv->rx_ring[rx_dma_owner_idx0] + sizeof(priv->rx_ring[rx_dma_owner_idx0]) - 1); // test-only
//	printf("%s (%d): sizeof_ring=%d sizeof_buf=%d\n", __func__, __LINE__, sizeof(priv->rx_ring[rx_dma_owner_idx0]), sizeof(priv->rx_buf[rx_dma_owner_idx0])); // test-only
#if 0
	rxd_info = (u32 *)KSEG1ADDR(
		&priv->rx_ring[rx_dma_owner_idx0].rxd_info2);
#else
	rxd_info = (u32 *)&priv->rx_ring[rx_dma_owner_idx0].rxd2;
#endif
//	printf("%s (%d): rxd_info=%08x @ %p\n", __func__, __LINE__, *rxd_info, rxd_info); // test-only

	// test-only: use accessor functions??? or just use value instead
	if ((*rxd_info & RX_DMA_DONE) == 0) {
//		printf("%s (%d): BIT 31 is 0\n", __func__, __LINE__); // test-only
		return -EAGAIN;
	}

	udelay(1); // test-only: needed???
	length = RX_DMA_GET_PLEN0(priv->rx_ring[rx_dma_owner_idx0].rxd2);
//	printf("%s (%d): length=%d\n", __func__, __LINE__, length); // test-only
	if (length >= MTK_QDMA_PAGE_SIZE)
		printf("%s (%d): length too big=%d !!!!!!!!!!!\n", __func__, __LINE__, length); // test-only

	if (length == 0) {
		printf("\n Warring!! Packet Length has error !!,In normal mode !\n");
	}

#if 0 // test-only: SP not defined in Linux driver
	if (priv->rx_ring[rx_dma_owner_idx0].rxd4.SP == 0) {
		// Packet received from CPU port
		printf("\n Normal Mode,Packet received from CPU port,plen=%d !!!!!!!!!!!!!!!!!!!!!!!! \n",length);
		//print_packet((void *)KSEG1ADDR(NetRxPackets[rx_dma_owner_idx0]),length);
//		inter_loopback_cnt++;
//		length = inter_loopback_cnt;//for return
		// test-only: return with error here??
		length = 0; // test-only: is this okay???
	} else {
#if 0
		printf("%s (%d): REAL RX here: rx_buf=%p KSEG=%x\n", __func__, __LINE__, priv->rx_buf[rx_dma_owner_idx0], KSEG1ADDR(priv->rx_buf[rx_dma_owner_idx0])); // test-only
#endif
#if 0
		*packetp = (uchar *)KSEG1ADDR(priv->rx_buf[rx_dma_owner_idx0]);
#else
		*packetp = priv->rx_buf[rx_dma_owner_idx0];
		invalidate_dcache_range((u32)*packetp, (u32)*packetp + length - 1); // test-only
#endif
//		NetReceive((void *)KSEG1ADDR(NetRxPackets[rx_dma_owner_idx0]), length );
//		print_hex_dump_bytes("RX: ", DUMP_PREFIX_OFFSET, *packetp, length); // test-only
	}
#else
#if 0
	printf("%s (%d): REAL RX here: rx_buf=%p KSEG=%x\n", __func__, __LINE__, priv->rx_buf[rx_dma_owner_idx0], KSEG1ADDR(priv->rx_buf[rx_dma_owner_idx0])); // test-only
#endif
#if 0
	*packetp = (uchar *)KSEG1ADDR(priv->rx_buf[rx_dma_owner_idx0]);
#else
	*packetp = priv->rx_buf[rx_dma_owner_idx0];
	invalidate_dcache_range((u32)*packetp, (u32)*packetp + length - 1); // test-only
#endif
//	NetReceive((void *)KSEG1ADDR(NetRxPackets[rx_dma_owner_idx0]), length );
//	print_hex_dump_bytes("RX: ", DUMP_PREFIX_OFFSET, *packetp, length); // test-only
#endif

#if 0
	rxd_info = (u32 *)&priv->rx_ring[rx_dma_owner_idx0].rxd_info4;
	*rxd_info = 0;

	rxd_info = (u32 *)&priv->rx_ring[rx_dma_owner_idx0].rxd_info2;
	*rxd_info = 0;
	priv->rx_ring[rx_dma_owner_idx0].rxd_info2.LS0 = 1;
#else
	priv->rx_ring[rx_dma_owner_idx0].rxd4 = 0;
	priv->rx_ring[rx_dma_owner_idx0].rxd2 = RX_DMA_LSO;
#endif

	wmb(); // test-only

	/* Tell the adapter where the TX/RX rings are located. */
	// test-only: each time???
//	RALINK_REG(RX_BASE_PTR0) = phys_to_bus((u32)&priv->rx_ring[0]);

//	printf("%s (%d): RX_CALC_IDX0=%d RX_DRX_IDX0=%d PDMA_INFO=%08x PDMA_GLO_CFG=%08x\n", __func__, __LINE__, RALINK_REG(RX_CALC_IDX0), RALINK_REG(RX_DRX_IDX0), RALINK_REG(PDMA_INFO), RALINK_REG(PDMA_GLO_CFG)); // test-only
	/*  Move point to next RXD which wants to alloc */
	RALINK_REG(RX_CALC_IDX0) = cpu_to_le32((u32)rx_dma_owner_idx0);

	/* Update to Next packet point that was received */
	rx_dma_owner_idx0 = (rx_dma_owner_idx0 + 1) % NUM_RX_DESC;

	return length;
}

static void mt76xx_eth_stop(struct udevice *dev)
{
	eth_dma_stop();
}


static int mt76xx_eth_probe(struct udevice *dev)
{
	struct mt76xx_eth_dev *priv = dev_get_priv(dev);
	struct mii_dev *bus;
	int ret;
	int i;

	/* Put rx and tx rings into KSEG1 area (uncached) */
	priv->tx_ring = (struct fe_tx_dma *)KSEG1ADDR(
		memalign(ARCH_DMA_MINALIGN,
			 sizeof(*priv->tx_ring) * NUM_TX_DESC));
	priv->rx_ring = (struct fe_rx_dma *)KSEG1ADDR(
		memalign(ARCH_DMA_MINALIGN,
			 sizeof(*priv->rx_ring) * NUM_RX_DESC));

	for (i = 0; i < NUM_RX_DESC; i++)
		priv->rx_buf[i] = memalign(PKTALIGN, MTK_QDMA_PAGE_SIZE);

#if 1 // test-only: whats this all about - check linux driver...
	//set clock resolution
//	extern unsigned long mips_bus_feq;
	u32 val;
	unsigned long mips_bus_feq = CONFIG_SYS_MIPS_TIMER_FREQ;
	val = le32_to_cpu(*(volatile u_long *)(RALINK_FRAME_ENGINE_BASE + 0x0008));
	val |= (mips_bus_feq / 1000000) << 8;
	*((volatile u_long *)(RALINK_FRAME_ENGINE_BASE + 0x0008)) =
		cpu_to_le32(val);
#endif

	/* Switch configuration */
	rt305x_esw_init();

	bus = mdio_alloc();
	if (!bus) {
		printf("Failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->read = mt76xx_mdio_read;
	bus->write = mt76xx_mdio_write;
	snprintf(bus->name, sizeof(bus->name), dev->name);

	ret = mdio_register(bus);
	if (ret)
		return ret;

	return 0;
}

static const struct eth_ops mt76xx_eth_ops = {
	.start		= mt76xx_eth_start,
	.send		= mt76xx_eth_send,
	.recv		= mt76xx_eth_recv,
	.stop		= mt76xx_eth_stop,
	.write_hwaddr	= mt76xx_eth_write_hwaddr,
};

static int mt76xx_eth_ofdata_to_platdata(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);

	pdata->iobase = devfdt_get_addr(dev);

	return 0;
}

static const struct udevice_id mt76xx_eth_ids[] = {
	{ .compatible = "mediatek,mt7622-eth" },
	{ }
};

U_BOOT_DRIVER(mt76xx_eth) = {
	.name	= "mt76xx_eth",
	.id	= UCLASS_ETH,
	.of_match = mt76xx_eth_ids,
	.ofdata_to_platdata = mt76xx_eth_ofdata_to_platdata,
	.probe	= mt76xx_eth_probe,
	.ops	= &mt76xx_eth_ops,
	.priv_auto_alloc_size = sizeof(struct mt76xx_eth_dev),
	.platdata_auto_alloc_size = sizeof(struct eth_pdata),
};
