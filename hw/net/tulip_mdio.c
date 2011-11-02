/*
 * QEMU Intel LXT971A 10/100 Mbps PHY MII Transceiver emulation
 *
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

#include "net/net.h"

#include "tulip_mdio.h"

void mii_reset(struct MiiTransceiver *s)
{
    s->state = MII_STATE_IDLE;
    s->bit_num = 0;
    s->cmd = 0;
    s->addr = 0;
    s->data = 0;
    s->last_clk = 0;
    s->last_mdio = 0;
    s->last_mdi = 0;
    s->selected_phy = 0;
}

int mii_tick(struct MiiTransceiver *s, int clk, int io)
{
    int ret;

    ret = s->last_mdio;

    if (s->last_clk == 0 && clk == 1) {
        switch (s->state) {
        case MII_STATE_IDLE:
            s->state = MII_STATE_READY;
            break;

        case MII_STATE_READY:
            if (io == 0 && s->last_mdi == 1) {
                s->state = MII_STATE_WAIT_START;
            }
            break;

        case MII_STATE_WAIT_START:
            if (io == 1) {
                s->state = MII_STATE_WAIT_CMD;
                s->bit_num = 0;
                s->cmd = 0;
            } else {
                s->state = MII_STATE_IDLE;
            }
            break;

        case MII_STATE_WAIT_CMD:
            s->cmd = (s->cmd << 1) | (io & 1);
            s->bit_num++;
            if (s->bit_num == 2) {
                s->state = MII_STATE_WAIT_PHYA;
                s->addr = 0;
                s->bit_num = 0;
            }
            break;

        case MII_STATE_WAIT_PHYA:
            s->addr = (s->addr << 1) | (io & 1);
            s->bit_num++;

            if (s->bit_num == 5) {
                s->selected_phy = s->addr;
                s->state = MII_STATE_WAIT_REGA;
                s->bit_num = 0;
                s->addr = 0;
            }

            break;

        case MII_STATE_WAIT_REGA:
            s->addr = (s->addr << 1) | (io & 1);
            s->bit_num++;

            if (s->bit_num == 5) {
                s->bit_num = 0;
                if (s->cmd == MII_CMD_READ) {
                    s->state = MII_STATE_READ_SYNC_CYCL;
                    if (s->selected_phy == s->taddr) {
                        s->data = s->readreg(s, s->addr);
                    } else {
                        s->data = 0xffff;
                    }
                }
                if (s->cmd == MII_CMD_WRITE) {
                    s->state = MII_STATE_WRITE_SYNC_CYCL;
                    s->data = 0;
                }
            }
            break;

        case MII_STATE_WRITE_SYNC_CYCL:
            s->bit_num++;

            if (s->bit_num == 2) {
                s->bit_num = 0;
                s->state = MII_STATE_GET_DATA;
                if (s->selected_phy == s->taddr) {
                    ret = 0;
                } else {
                    ret = 1;
                }
            }
            break;

        case MII_STATE_READ_SYNC_CYCL:
            s->bit_num++;

            if (s->bit_num == 1) {
                s->bit_num = 0;
                s->state = MII_STATE_PUT_DATA;
                ret = 0;
            }
            break;

        case MII_STATE_PUT_DATA:
            ret = ((s->data >> (15 - s->bit_num)) & 1);
            s->bit_num++;
            if (s->bit_num == 16) {
                s->state = MII_STATE_IDLE;
            }
            break;

        case MII_STATE_GET_DATA:
            s->data = (s->data << 1) | (io & 1);
            s->bit_num++;
            if (s->bit_num == 16) {
                s->state = MII_STATE_IDLE;
                /* FIXME: check phy_addr */
                s->writereg(s, s->addr, s->data);
            }
            break;
        }
        s->last_mdio = ret;
        s->last_mdi = io;
    }

    s->last_clk = clk;

    return ret;
}

static uint16_t lxt971_readreg(struct MiiTransceiver *s, int index)
{
    return s->mii_data[index];
}

static void lxt971_writereg(struct MiiTransceiver *s, int index, uint16_t val)
{
    s->mii_data[index] = val;
}

static void lxt971_reset(struct MiiTransceiver *s)
{
    uint16_t a[] = {
    0x1000, 0x782d, 0x0013, 0x78e2, 0x01e1, 0x45e1, 0x0007, 0x2001,
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    0x0084, 0x4780, 0x0000, 0x00f4, 0x0422, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x00c8, 0x0000, 0xffff, 0x0000, 0x0000, 0x3678,
    };
    int i;

    mii_reset(s);

    for (i = 0; i < 32; i++) {
        s->mii_data[i] = a[i];
    }
}

void lxt971_init(struct MiiTransceiver *s, int phy_addr)
{
    s->reset = lxt971_reset;
    s->readreg = lxt971_readreg;
    s->writereg = lxt971_writereg;

    s->taddr = phy_addr;

    lxt971_reset(s);
}
