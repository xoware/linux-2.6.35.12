#ifndef  VSC7385_H
#define  VSC7385_H

int vsc7385_reg_read(u32 block, u32 subblock, u32 addr, u32 *value);
int vsc7385_reg_write(u32 block, u32 subblock, u32 addr, u32 value);

/* VSC7385 register addressing */
#define BLOCK_SYSTEM       7
#define SUBBLOCK_SYSTEM    0
#define ADDRESS_GMIIDELAY  5
#define ADDRESS_ICPU_CTRL  0x10
#define ADDRESS_ICPU_ADDR  0x11
#define ADDRESS_ICPU_DATA  0x12
#define ADDRESS_GLORESET   0x14

#define BLOCK_MEMINIT      3
#define SUBBLOCK_MEMINIT   2
#define ADDRESS_MEMINIT    0

#define BLOCK_FRAME_ANALYZER    2
#define SUBBLOCK_FRAME_ANALYZER 0
#define ADDRESS_RECVMASK        0x10
#define ADDRESS_MACACCESS       0xb0
#define ADDRESS_VLANACCESS      0xe0

#define BLOCK_MAC          1
#define ADDRESS_MAC_CFG    0
#define ADDRESS_ADVPORTM   0x19

#define BLOCK_MII 	3
#define SUBBLOCK_PHY 	0
#define REG_MIIMCMD 	1
#define REG_MIIMDATA 	2
#define MIICMD_OPER_RD 	1
#define MIICMD_OPER_WR 	0
// 0000 01XX XXX0 0001 0000 0000 0000 0000
#define PHY_ADDR(X) 	(MIICMD_OPER_RD<<26)|(X<<21)
#define PHY_REG(X) 	(X<<16)
#define PHY_STAT 	1
#define PHY_AUX_CTRL_STAT 	0x1C
enum LANX { LAN0, LAN1, LAN2, LAN3, LAN4 };
// PHY_REG(PHY_STAT) | PHY_ADDR(LAN3)
#define LNKS_MSK 	0x4
#define AUTONEG_MSK 	0x8000
#define SPEEDS_MSK 	0x0018
#define FDXS_MSK 	0x0020
#define HD10B		0x0 << 3
#define FD10B		0x4 << 3
#define HD100B		0x1 << 3
#define FD100B		0x5 << 3
#define HD1000B		0x2 << 3
#define FD1000B		0x6 << 3


/* ethernet modes */
enum { LinkDown, Link1000Full, Link100Full, Link10Full, Link100Half, Link10Half };

#define MAC_RESET     0x20000030
#define MAC_1000_FULL 0x10070180
#define MAC_100_FULL  0x10050440
#define MAC_10_FULL   0x10050440
#define MAC_100_HALF  0x90010440
#define MAC_10_HALF   0x90010440

#define MAC_EXT_CLK_1000 0x1
#define MAC_EXT_CLK_100  0x2
#define MAC_EXT_CLK_10   0x3
#define MAC_INT_CLK      0x4


extern unsigned char vsc7385fw[];
extern int  vsc7385fw_len;

#endif  // VSC7385_H
