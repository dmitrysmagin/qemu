/*
 * QEMU DEC 21143 (Tulip) emulation
 *
 * Copyright (C) 2011, 2013 Antony Pavlov
 * Copyright (C) 2013 Dmitry Smagin
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

#ifndef HW_TULIP_H
#define HW_TULIP_H

#include "tulip_mdio.h"

#define TULIP_CSR_REGION_SIZE       0x80

typedef struct TulipState_st {
    NICState *nic;
    NICConf conf;
    QEMUTimer *timer;
    MemoryRegion mmio;

    qemu_irq irq;

    void (*phys_mem_read)(void *dma_opaque, hwaddr addr,
                          uint8_t *buf, int len);
    void (*phys_mem_write)(void *dma_opaque, hwaddr addr,
                           uint8_t *buf, int len);
    void *dma_opaque;

    eeprom_t *eeprom;
    struct MiiTransceiver mii;

    uint32_t mac_reg[TULIP_CSR_REGION_SIZE >> 2];

    uint32_t cur_tx_desc;
    uint32_t cur_rx_desc;
    int tx_polling;
} TulipState;

int tulip_init(DeviceState *dev, TulipState *s, NetClientInfo *info);
void tulip_cleanup(TulipState *s);
void tulip_reset(void *opaque);

void tulip_timer(void *opaque);
uint64_t tulip_csr_read(void *opaque, hwaddr addr, unsigned size);
void tulip_csr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
int tulip_can_receive(NetClientState *nc);
ssize_t tulip_receive(NetClientState *nc, const uint8_t *buf, size_t size);
void tulip_set_link_status(NetClientState *nc);

#endif
