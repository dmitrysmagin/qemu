/*
 * vim:ts=4:sw=4:softtabstop=4:smarttab:expandtab
 *
 * QEMU 93LC46B uWire-connected EEPROM emulation
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

#ifndef HW_NET_TULIP_UWIRE_EEPROM_H
#define HW_NET_TULIP_UWIRE_EEPROM_H

struct MicrowireEeprom {
    /* 93LC46B */
    int state;
#define MW_EEPROM_STATE_IDLE        0
#define MW_EEPROM_STATE_WAIT_CMD    1
#define MW_EEPROM_STATE_WAIT_ADDR   2
#define MW_EEPROM_STATE_WAIT_DATA   3
#define MW_EEPROM_STATE_OUT_DATA    4
#define MW_EEPROM_STATE_ERROR       5

    int last_clk;
    int bit_num;
    int cmd;
#define MW_EEPROM_CMD_ERASE         7
#define MW_EEPROM_CMD_READ          6
#define MW_EEPROM_CMD_WRITE         5
#define MW_EEPROM_CMD_ERASE_ALL     4

    int addr;
    int data;

    uint16_t eeprom_data[64];
};

/* EEPROM Commands - Microwire */
#define EEPROM_READ_OPCODE_MICROWIRE  0x6  /* EEPROM read opcode */
#define EEPROM_WRITE_OPCODE_MICROWIRE 0x5  /* EEPROM write opcode */
#define EEPROM_ERASE_OPCODE_MICROWIRE 0x7  /* EEPROM erase opcode */
#define EEPROM_EWEN_OPCODE_MICROWIRE  0x13 /* EEPROM erase/write enable */
#define EEPROM_EWDS_OPCODE_MICROWIRE  0x10 /* EEPROM erast/write disable */

extern int microwire_tick(struct MicrowireEeprom *s, int clk, int en, int din);
extern void microwire_reset(struct MicrowireEeprom *s);
extern void microwire_load_data(struct MicrowireEeprom *s,
    const uint16_t *data);

#endif /* HW_NET_TULIP_UWIRE_EEPROM_H */
