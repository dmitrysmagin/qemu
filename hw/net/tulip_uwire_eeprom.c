/*
 * QEMU Microchip 93LC46B 1K Microwire Compatible Serial EEPROM emulation
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

#include "tulip_uwire_eeprom.h"

void microwire_reset(struct MicrowireEeprom *s)
{
    s->state = MW_EEPROM_STATE_IDLE;
    s->bit_num = 0;
    s->cmd = 0;
    s->addr = 0;
    s->data = 0;
    s->last_clk = 0;
}

void microwire_load_data(struct MicrowireEeprom *s, const uint16_t *data)
{
    memmove(s->eeprom_data, data, sizeof(s->eeprom_data));
}

int microwire_tick(struct MicrowireEeprom *s, int clk, int en, int din)
{
    int ret;

    ret = 0;

    /* Microwire eeprom disabled */
    if (en == 0) {
        s->state = MW_EEPROM_STATE_IDLE;
        return 0;
    }

    if (s->last_clk == 0 && clk == 1) {
        switch (s->state) {
        case MW_EEPROM_STATE_IDLE:
            /*
             * The Start bit is detected by the device if CS and DI are
             * both high with respect to the positive edge of CLK for
             * the first time.
             */
            if (!din) {
                break;
            }

            s->state = MW_EEPROM_STATE_WAIT_CMD;
            s->cmd = din & 1;
            s->bit_num = 0;
            break;

        case MW_EEPROM_STATE_WAIT_CMD:
            s->cmd = (s->cmd << 1) | (din & 1);
            s->bit_num++;
            if (s->bit_num == 2) {
                if (s->cmd == 0) {
                    s->state = MW_EEPROM_STATE_IDLE;
                }
                if (s->cmd == MW_EEPROM_CMD_READ) {
                    s->state = MW_EEPROM_STATE_WAIT_ADDR;
                    s->addr = 0;
                    s->bit_num = 0;
                }
            }
            break;

        case MW_EEPROM_STATE_WAIT_ADDR:
            s->addr = (s->addr << 1) | (din & 1);
            s->bit_num++;
            if (s->bit_num == 6) { /* ROM width = 6 */
                if (s->cmd == MW_EEPROM_CMD_READ) {
                    s->state = MW_EEPROM_STATE_OUT_DATA;
                    s->bit_num = 0;
                }
            }
            break;

        case MW_EEPROM_STATE_OUT_DATA:
            s->bit_num++;
            if (s->bit_num == 16) {
                if (s->cmd == MW_EEPROM_CMD_READ) {
                    s->state = MW_EEPROM_STATE_IDLE;
                }
            }

            ret = ((s->eeprom_data[s->addr] >> (16 - s->bit_num)) & 1);

            break;
        }
    }

    s->last_clk = clk;

    return ret;
}
