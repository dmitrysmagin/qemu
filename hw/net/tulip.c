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

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "qemu/timer.h"

#include "hw/nvram/eeprom93xx.h"
#include "tulip_mdio.h"

#define TULIP_CSR0  0x00
 #define TULIP_CSR0_SWR     0x01
 #define TULIP_CSR0_TAP_MASK    (0x03 << 17)
#define TULIP_CSR1  0x08
#define TULIP_CSR2  0x10
#define TULIP_CSR3  0x18
#define TULIP_CSR4  0x20
#define TULIP_CSR5  0x28
 #define TULIP_CSR5_TI         (1 <<  0) /* Transmit Interrupt */
 #define TULIP_CSR5_RI         (1 <<  6) /* Receive Interrupt */
 #define TULIP_CSR5_NIS        (1 << 16) /* Or'ed bits 0, 2, 6, 11, 14  */
 #define TULIP_CSR5_NIS_SHIFT  (16)
 #define TULIP_CSR5_RS_MASK    (7 << 17)
 #define TULIP_CSR5_TS_MASK    (7 << 20)

#define TULIP_CSR6  0x30 /* Operation Mode Register */
 #define TULIP_CSR6_TXON    0x2000
 #define TULIP_CSR6_RXON    0x0002

#define TULIP_CSR7  0x38 /* Interrupt Enable Register */
#define TULIP_CSR8  0x40
#define TULIP_CSR9  0x48 /* Boot ROM, Serial ROM, and MII Management Register */
 #define TULIP_CSR9_MDI         (1 << 19) /* MDIO Data In */
 #define TULIP_CSR9_MDOM        (1 << 18) /* MDIO Operation Mode */
 #define TULIP_CSR9_MDO         (1 << 17) /* MDIO Data Out */
 #define TULIP_CSR9_MDC         (1 << 16) /* MDIO Clock */
 #define TULIP_CSR9_RD          (1 << 14)
 #define TULIP_CSR9_WR          (1 << 13)
 #define TULIP_CSR9_SR          (1 << 11) /* Serial ROM Select */
 #define TULIP_CSR9_SRDO        (1 << 3) /* Serial ROM Data Out */
 #define TULIP_CSR9_SRDI        (1 << 2) /* Serial ROM Data In */
 #define TULIP_CSR9_SRCK        (1 << 1) /* Serial ROM Clock */
 #define TULIP_CSR9_SRCS        (1) /* Serial ROM Chip Select */
#define TULIP_CSR10 0x50
#define TULIP_CSR11 0x58

#define TULIP_CSR12 0x60

#define TULIP_CSR13 0x68

#define TULIP_CSR14 0x70 /* SIA Transmit and Receive Register */
#define TULIP_CSR15 0x78

#define defreg(x)   x = (TULIP_##x>>2)
enum {
    defreg(CSR0),
    defreg(CSR1),
    defreg(CSR2),
    defreg(CSR3),
    defreg(CSR4),
    defreg(CSR5),
    defreg(CSR6),
    defreg(CSR7),
    defreg(CSR8),
    defreg(CSR9),
    defreg(CSR10),
    defreg(CSR11),
    defreg(CSR12),
    defreg(CSR13),
    defreg(CSR14),
    defreg(CSR15),
};

/* The Tulip Receive Descriptor */
struct tulip_rx_desc {
    uint32_t status;
    uint32_t length;
    uint32_t buffer1;
    uint32_t buffer2;
};

#define TULIP_RDES0_OWN   (1 << 31)
#define TULIP_RDES0_FS    (1 <<  9) /* First descriptor */
#define TULIP_RDES0_LS    (1 <<  8) /* Last descriptor */

/* The Tulip Transmit Descriptor */
struct tulip_tx_desc {
    uint32_t status;
    uint32_t length;
    uint32_t buffer1;
    uint32_t buffer2;  /* Linux use only buffer 1. */
};

#define TULIP_TDES0_OWN   (1 << 31)

#define TULIP_TDES1_IC    (1 << 31) /* Interrupt on Completion */
#define TULIP_TDES1_LS    (1 << 30) /* Last Segment */
#define TULIP_TDES1_FS    (1 << 29) /* First Segment */
#define TULIP_TDES1_SET   (1 << 27) /* Setup Packet */

#define TULIP_CSR_REGION_SIZE       0x80

typedef struct TulipState_st {
    PCIDevice dev;
    NICState *nic;
    NICConf conf;
    MemoryRegion mmio;
    MemoryRegion io;
    QEMUTimer *timer;
    qemu_irq irq;

    eeprom_t *eeprom;
    struct MiiTransceiver mii;

    uint32_t mac_reg[TULIP_CSR_REGION_SIZE >> 2];

    uint32_t cur_tx_desc;
    uint32_t cur_rx_desc;
    int tx_polling;
} TulipState;

#define EEPROM_SIZE 64
#define EEPROM_MACADDR_OFFSET 10
/*
 * DEC has developed its own EEPROM format
 * see Digital Semiconductor 21X4 Serial ROM Format ver. 4.05 2-Mar-1998
 * for details.
 *
 * Also see tulip-diag utility code from nictools-pci package
 *     http://ftp.debian.org/debian/pool/main/n/nictools-pci/
 */
static const
uint16_t tulip_eeprom_template[EEPROM_SIZE] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
 /* ^^^^^^^^^^^^^^ Subsystem IDs */
    0x0000, 0x0104, 0x5554, 0x494c, 0x0050, 0x1e00, 0x0000, 0x0800,
                 /* ^^^^^^^^^^^^^^^^^^^^^^ MAC address here */
    0x8d01, 0x0003, 0x0000, 0x7800, 0x01e0, 0x5000, 0x1800, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
     /* we have no Magic Packet block, so checksum is here  ^^^^^^ */
};

static void tulip_reset(void *opaque);
static void tulip_update_irq(TulipState *s);

static inline uint32_t mac_readreg(TulipState *s, int index)
{
    return s->mac_reg[index];
}

static inline void mac_writereg(TulipState *s, int index, uint32_t val)
{
    s->mac_reg[index] = val;
}

/* FIXME: we use external MII tranceiver, so we must change it status here */
static void tulip_set_link_status(NetClientState *nc)
{
}

static int tulip_can_receive(NetClientState *nc)
{
    TulipState *s = qemu_get_nic_opaque(nc);

    return mac_readreg(s, CSR6) & TULIP_CSR6_RXON;
}

static ssize_t tulip_receive(NetClientState *nc, const uint8_t *buf,
                             size_t size)
{
    TulipState *s = qemu_get_nic_opaque(nc);
    struct tulip_rx_desc desc;
    uint32_t status, buffer1, buffer2;
    uint32_t cur_rx_desc;

    if (!(mac_readreg(s, CSR6) & TULIP_CSR6_RXON)) {
        return -1;
    }

    cur_rx_desc = s->cur_rx_desc;
    while (1) {
        pci_dma_read(&s->dev, cur_rx_desc, (void *)&desc, sizeof(desc));
        status = le32_to_cpu(desc.status);
        buffer1 = le32_to_cpu(desc.buffer1);
        buffer2 = le32_to_cpu(desc.buffer2);

        /* FIXME: check desc.length too */
        if (!(status & TULIP_RDES0_OWN)) {
            return -1;
        }

        desc.status = cpu_to_le32(
            (((size + 4) << 16) & 0x3fff0000)
            | TULIP_RDES0_FS | TULIP_RDES0_LS);

        pci_dma_write(&s->dev, buffer1, buf, size);
        pci_dma_write(&s->dev, cur_rx_desc, (void *)&desc, sizeof(desc));
        mac_writereg(s, CSR5, TULIP_CSR5_RI | mac_readreg(s, CSR5));

        s->cur_rx_desc = buffer2;
        break;
    }

    tulip_update_irq(s);

    return size;
}

static inline void tulip_csr0_write(TulipState *s, uint32_t val)
{
    if (val & TULIP_CSR0_SWR) {
        tulip_reset(s);
        return;
    }

    if (val & TULIP_CSR0_TAP_MASK) {
        s->tx_polling = 1;
    } else {
        s->tx_polling = 0;
    }

    mac_writereg(s, CSR0, val);
}

static inline void tulip_csr5_write(TulipState *s, uint32_t val)
{
    uint32_t csr5_write_mask = 0x02fe0000;

    val = val & (~csr5_write_mask);

    mac_writereg(s, CSR5, mac_readreg(s, CSR5) & (~val));
}

static inline void tulip_csr6_write(TulipState *s, uint32_t val)
{
    mac_writereg(s, CSR6, val);
}

static inline void tulip_csr9_write(TulipState *s, uint32_t val)
{
    if (val & TULIP_CSR9_SR) { /* Serial ROM */
        int srdo;
        int srcs = ((val & TULIP_CSR9_SRCS) != 0);
        int srck = ((val & TULIP_CSR9_SRCK) != 0);
        int srdi = ((val & TULIP_CSR9_SRDI) != 0);

        eeprom93xx_write(s->eeprom, srcs, srck, srdi);

        srdo = eeprom93xx_read(s->eeprom);
        if (srdo) {
            mac_writereg(s, CSR9, val | TULIP_CSR9_SRDO);
        } else {
            mac_writereg(s, CSR9, val & ~TULIP_CSR9_SRDO);
        }
    } else {
        int mdo;

        mdo = mii_tick(&s->mii, (val & TULIP_CSR9_MDC) >> 16,
                        (val & TULIP_CSR9_MDO) >> 17);

        if (val & TULIP_CSR9_MDOM) {
            if (mdo) {
                val |= TULIP_CSR9_MDI;
            } else {
                val &= ~TULIP_CSR9_MDI;
            }
        }
        mac_writereg(s, CSR9, val);
    }
}

/* Just now we don't use MAC-address filtering (we always use promisc mode),
   so just drop any setup frame */
static void process_setup_frame(TulipState *s, uint8_t *a)
{
}

static void process_tx(TulipState *s)
{
    struct tulip_tx_desc desc;
    uint32_t status, length, buffer1, buffer2;
    uint32_t cur_tx_desc;

    uint8_t a[1600];
    int cur_offset;
    int frame_length;

    cur_offset = 0;
    frame_length = 0;

    cur_tx_desc = s->cur_tx_desc;

    while (1) {
        uint32_t to_copy;

        pci_dma_read(&s->dev, cur_tx_desc, (void *)&desc, sizeof(desc));
        status = le32_to_cpu(desc.status);
        length = le32_to_cpu(desc.length);
        buffer1 = le32_to_cpu(desc.buffer1);
        buffer2 = le32_to_cpu(desc.buffer2);

        if (!(status & TULIP_TDES0_OWN)) {
            break;
        }

        to_copy = 0x7ff & length;

        /* First Segment */
        if (length & TULIP_TDES1_FS) {
            cur_offset = 0;
            frame_length = to_copy;
        }

        if (length & TULIP_TDES1_SET) { /* Setup Frame */
            /* FIXME: check (to_copy == 192) */
            pci_dma_read(&s->dev, buffer1, (void *)a, to_copy);

            process_setup_frame(s, a);
        } else {

            pci_dma_read(&s->dev, buffer1, (void *)(a + cur_offset), to_copy);

            cur_offset += to_copy;

            /* ! First Segment */
            if (!(length & TULIP_TDES1_FS)) {
                frame_length += to_copy;
            }

            /* Last Segment */
            if (length & TULIP_TDES1_LS) {
                qemu_send_packet(qemu_get_queue(s->nic), a, frame_length);
            }
        }

        desc.status = cpu_to_le32(status & ~TULIP_TDES0_OWN);
        pci_dma_write(&s->dev, cur_tx_desc, (void *)&desc, sizeof(desc));

        /* Last Segment */
        if (length & TULIP_TDES1_LS) {
            /* FIXME: check IC */
            /* FIXME: mac_writereg -> tulip_csr5_write */
            mac_writereg(s, CSR5, TULIP_CSR5_TI | mac_readreg(s, CSR5));
        }

        cur_tx_desc = buffer2;
        s->cur_tx_desc = cur_tx_desc;
    }
}

static void tulip_csr_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    TulipState *s = opaque;
    unsigned int index = addr & (TULIP_CSR_REGION_SIZE - 1);

    switch (index) {
    case TULIP_CSR0: /* Bus Mode Register */
        tulip_csr0_write(s, (uint32_t)val);
        break;

    case TULIP_CSR1: /* Transmit Poll Demand/Current Descriptor Address */
        if (mac_readreg(s, CSR6) & TULIP_CSR6_TXON) {
            process_tx(s);
            tulip_update_irq(s);
        }
        break;

    case TULIP_CSR2: /* Receive Poll Demand/Current Descriptor Address */
        break;

    case TULIP_CSR3: /* Start of Receive List */
        mac_writereg(s, CSR3, val);
        s->cur_rx_desc = val;
        break;

    case TULIP_CSR4: /* Start of Transmit List */
        mac_writereg(s, CSR4, val);
        s->cur_tx_desc = val;
        break;

    case TULIP_CSR5: /* Status Register */
        tulip_csr5_write(s, (uint32_t)val);
        tulip_update_irq(s);
        break;

    case TULIP_CSR6: /* Command/Mode Register */
        tulip_csr6_write(s, (uint32_t)val);
        break;

    case TULIP_CSR9: /* Boot/Serial ROM and MII Management Register */
        tulip_csr9_write(s, (uint32_t)val);
        break;

    case TULIP_CSR13:
        mac_writereg(s, CSR13, val);
        break;

    case TULIP_CSR14:
    case TULIP_CSR15:
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
            "tulip: write access to unknown register 0x" TARGET_FMT_plx, addr);
    }
}

static uint64_t tulip_csr_read(void *opaque, hwaddr addr, unsigned size)
{
    TulipState *s = opaque;
    unsigned int index = addr & (TULIP_CSR_REGION_SIZE - 1);
    uint64_t ret;

    ret = 0;

    switch (index) {
    case TULIP_CSR0: /* Bus Mode Register */
    case TULIP_CSR5: /* Status Register */
    case TULIP_CSR6: /* Command/Mode Register */
    case TULIP_CSR8:
    case TULIP_CSR9: /* Boot/Serial ROM and MII Management Register */
    case TULIP_CSR12:
    case TULIP_CSR13:
    /* FIXME: tulip_csr_read: unimplemented register index = 70 */
    case TULIP_CSR14:
    case TULIP_CSR15:
        ret = (uint64_t)mac_readreg(s, (index >> 2));
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
            "tulip: read access to unknown register 0x" TARGET_FMT_plx, addr);

    }

    return ret;
}

static const MemoryRegionOps tulip_mmio_ops = {
    .read = tulip_csr_read,
    .write = tulip_csr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_tulip = {
    .name = "tulip",
    .version_id = 2,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, TulipState),
        VMSTATE_MACADDR(conf.macaddr, TulipState),
        VMSTATE_END_OF_LIST()
    }
};

static const uint32_t mac_reg_init[] = {
    [CSR0] = 0xfe000000,
    [CSR1] = 0xffffffff,
    [CSR2] = 0x00000000,
    [CSR3] = 0x00000000,
    [CSR4] = 0x00000000,
    [CSR5] = 0xf0000000,
    [CSR6] = 0x00000000,
    [CSR7] = 0x00000000,
    [CSR8] = 0xe0000000,
    [CSR9] = 0xfff483ff,
    [CSR10] = 0x00000000,
    [CSR11] = 0x00000000,
    [CSR12] = 0x00000000,
    [CSR13] = 0x00000000,
    [CSR14] = 0x00000000,
    [CSR15] = 0x00000000,
};

static void tulip_cleanup(NetClientState *nc)
{
    TulipState *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static void pci_tulip_uninit(PCIDevice *dev)
{
    TulipState *d = DO_UPCAST(TulipState, dev, dev);

    qemu_free_irq(d->irq);
    memory_region_destroy(&d->mmio);
    memory_region_destroy(&d->io);
    timer_del(d->timer);
    timer_free(d->timer);
    eeprom93xx_free(&dev->qdev, d->eeprom);
    qemu_del_nic(d->nic);
}

static void tulip_reset(void *opaque)
{
    TulipState *d = opaque;

    memset(d->mac_reg, 0, sizeof d->mac_reg);
    memmove(d->mac_reg, mac_reg_init, sizeof mac_reg_init);

    d->tx_polling = 0;
}

static NetClientInfo net_tulip_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .can_receive = tulip_can_receive,
    .receive = tulip_receive,
    .cleanup = tulip_cleanup,
    .link_status_changed = tulip_set_link_status,
};

static void tulip_update_irq(TulipState *s)
{
    int isr = 0;

    /* calculate NIS (sum of normal interrupts) */
    s->mac_reg[CSR5] &= ~TULIP_CSR5_NIS;

    if (s->mac_reg[CSR5] & (TULIP_CSR5_TI | TULIP_CSR5_RI)) {
        s->mac_reg[CSR5] |= TULIP_CSR5_NIS;
        isr = 1;
    }

    /* FIXME: when calculating NIS take into account masked interupts */

    /* FIXME: can we report about Abnormal Interrupt Summary too? */

    qemu_set_irq(s->irq, isr);
}

static void tulip_timer(void *opaque)
{
    TulipState *s = opaque;

    if (s->tx_polling) {
        process_tx(s);
    }

    tulip_update_irq(s);

    timer_mod(s->timer,
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + get_ticks_per_sec() / 10);
}

static int pci_tulip_init(PCIDevice *pci_dev)
{
    TulipState *d = DO_UPCAST(TulipState, dev, pci_dev);
    uint8_t *pci_conf;
    uint16_t *eeprom_contents;
    uint8_t *macaddr;
    int i;

    pci_conf = d->dev.config;

    /* TODO: RST# value should be 0, PCI spec 6.2.4 */
    pci_conf[PCI_CACHE_LINE_SIZE] = 0x10;

    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    /* PCI interface */
    memory_region_init_io(&d->mmio, OBJECT(d), &tulip_mmio_ops, d,
                          "tulip-mmio", TULIP_CSR_REGION_SIZE);
    memory_region_init_io(&d->io, OBJECT(d), &tulip_mmio_ops, d,
                          "tulip-io", TULIP_CSR_REGION_SIZE);

    pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->io);
    pci_register_bar(&d->dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);

    d->irq = pci_allocate_irq(pci_dev);

    lxt971_init(&d->mii, 0);

    qemu_macaddr_default_if_unset(&d->conf.macaddr);
    macaddr = d->conf.macaddr.a;

    d->eeprom = eeprom93xx_new(&pci_dev->qdev, EEPROM_SIZE);
    eeprom_contents = eeprom93xx_data(d->eeprom);
    memmove(eeprom_contents, tulip_eeprom_template,
            EEPROM_SIZE * sizeof(uint16_t));

    /* copy macaddr to eeprom as linux driver want find it there */
    for (i = 0; i < 3; i++) {
        eeprom_contents[EEPROM_MACADDR_OFFSET + i] =
                (macaddr[2 * i + 1] << 8) | macaddr[2 * i];
    }

    /*
     * FIXME: we have to update eeprom checksum too.
     *
     * see tulip-diag utility code from nictools-pci package
     *   http://ftp.debian.org/debian/pool/main/n/nictools-pci/
     */

    d->nic = qemu_new_nic(&net_tulip_info, &d->conf,
                          object_get_typename(OBJECT(pci_dev)),
                          d->dev.qdev.id, d);

    qemu_format_nic_info_str(qemu_get_queue(d->nic), macaddr);

    add_boot_device_path(d->conf.bootindex, &pci_dev->qdev, "/ethernet-phy@0");

    d->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tulip_timer, d);
    timer_mod(d->timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + get_ticks_per_sec());

    return 0;
}

static void qdev_tulip_reset(DeviceState *dev)
{
    TulipState *d = DO_UPCAST(TulipState, dev.qdev, dev);

    tulip_reset(d);
}

static Property tulip_properties[] = {
    DEFINE_NIC_PROPERTIES(TulipState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void tulip_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = pci_tulip_init;
    k->exit = pci_tulip_uninit;
    k->vendor_id = PCI_VENDOR_ID_DEC;
    k->device_id = PCI_DEVICE_ID_DEC_21142;
    k->revision = 0x41; /* 21143 chip */
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->desc = "DEC 21143 Tulip";
    dc->reset = qdev_tulip_reset;
    dc->vmsd = &vmstate_tulip;
    dc->props = tulip_properties;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo tulip_info = {
    .name = "tulip",
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(TulipState),
    .class_init = tulip_class_init,
};

static void tulip_register_types(void)
{
    type_register_static(&tulip_info);
}

type_init(tulip_register_types)
