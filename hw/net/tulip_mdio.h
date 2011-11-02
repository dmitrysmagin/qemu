/*
 * vim:ts=4:sw=4:softtabstop=4:smarttab:expandtab
 *
 * QEMU LXT971A MII Transceiver emulation
 * Copyright (C) 2011, 2013 Antony Pavlov
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_NET_TULIP_MDIO_H
#define HW_NET_TULIP_MDIO_H

struct MiiTransceiver {
    int state;
#define MII_STATE_IDLE        0
#define MII_STATE_READY       1
#define MII_STATE_WAIT_START  2
#define MII_STATE_WAIT_CMD    3
#define MII_STATE_WAIT_PHYA   4
#define MII_STATE_WAIT_REGA   5
#define MII_STATE_READ_SYNC_CYCL   6
#define MII_STATE_WRITE_SYNC_CYCL  7
#define MII_STATE_PUT_DATA    8
#define MII_STATE_GET_DATA    9
#define MII_STATE_ERROR       10

    int taddr;

    int last_mdio;
    int last_mdi;
    int last_clk;
    int bit_num;
    int selected_phy;

    int cmd;
#define MII_CMD_READ          2
#define MII_CMD_WRITE         1

    int addr;
    int data;

    uint16_t mii_data[32];

    void (*reset)(struct MiiTransceiver *s);
    uint16_t (*readreg)(struct MiiTransceiver *, int);
    void (*writereg)(struct MiiTransceiver *, int, uint16_t);
};

/* PHY Registers defined by IEEE */
#define PHY_CTRL         0x00 /* Control Register */
#define PHY_STATUS       0x01 /* Status Regiser */
#define PHY_ID1          0x02 /* Phy Id Reg (word 1) */
#define PHY_ID2          0x03 /* Phy Id Reg (word 2) */
#define PHY_AUTONEG_ADV  0x04 /* Autoneg Advertisement */
#define PHY_LP_ABILITY   0x05 /* Link Partner Ability (Base Page) */
#define PHY_AUTONEG_EXP  0x06 /* Autoneg Expansion Reg */
#define PHY_NEXT_PAGE_TX 0x07 /* Next Page TX */
#define PHY_LP_NEXT_PAGE 0x08 /* Link Partner Next Page */
#define PHY_1000T_CTRL   0x09 /* 1000Base-T Control Reg */
#define PHY_1000T_STATUS 0x0A /* 1000Base-T Status Reg */
#define PHY_EXT_STATUS   0x0F /* Extended Status Reg */

#define MAX_PHY_REG_ADDRESS        0x1F  /* 5 bit address bus (0-0x1F) */
#define MAX_PHY_MULTI_PAGE_REG     0xF   /* Registers equal on all pages */

/* PHY Status Register */
#define MII_SR_EXTENDED_CAPS     0x0001 /* Extended register capabilities */
#define MII_SR_JABBER_DETECT     0x0002 /* Jabber Detected */
#define MII_SR_LINK_STATUS       0x0004 /* Link Status 1 = link */
#define MII_SR_AUTONEG_CAPS      0x0008 /* Auto Neg Capable */
#define MII_SR_REMOTE_FAULT      0x0010 /* Remote Fault Detect */
#define MII_SR_AUTONEG_COMPLETE  0x0020 /* Auto Neg Complete */
#define MII_SR_PREAMBLE_SUPPRESS 0x0040 /* Preamble may be suppressed */
#define MII_SR_EXTENDED_STATUS   0x0100 /* Ext. status info in Reg 0x0F */
#define MII_SR_100T2_HD_CAPS     0x0200 /* 100T2 Half Duplex Capable */
#define MII_SR_100T2_FD_CAPS     0x0400 /* 100T2 Full Duplex Capable */
#define MII_SR_10T_HD_CAPS       0x0800 /* 10T   Half Duplex Capable */
#define MII_SR_10T_FD_CAPS       0x1000 /* 10T   Full Duplex Capable */
#define MII_SR_100X_HD_CAPS      0x2000 /* 100X  Half Duplex Capable */
#define MII_SR_100X_FD_CAPS      0x4000 /* 100X  Full Duplex Capable */
#define MII_SR_100T4_CAPS        0x8000 /* 100T4 Capable */

extern void mii_reset(struct MiiTransceiver *s);
extern int mii_tick(struct MiiTransceiver *s, int clk, int io);
extern void lxt971_init(struct MiiTransceiver *s, int phy_addr);

#endif /* HW_NET_TULIP_MDIO_H */
