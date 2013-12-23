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
#include "hw/qdev.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "qemu/timer.h"

#include "hw/nvram/eeprom93xx.h"
#include "tulip.h"
#include "tulip_mdio.h"

#define TYPE_PCI_TULIP "tulip"

#define PCI_TULIP(obj) \
    OBJECT_CHECK(PCITulipState, (obj), TYPE_PCI_TULIP)

typedef struct {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    TulipState state;
    MemoryRegion io_bar;
} PCITulipState;

static const VMStateDescription vmstate_tulip = {
    .name = "tulip",
    .version_id = 2,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, PCITulipState),
        VMSTATE_MACADDR(state.conf.macaddr, PCITulipState),
        /* FIXME: add VMSTATE_STRUCT for saving state */
        VMSTATE_END_OF_LIST()
    }
};

/* PCI Interfaces */

static const MemoryRegionOps tulip_mmio_ops = {
    .read = tulip_csr_read,
    .write = tulip_csr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void pci_physical_memory_write(void *dma_opaque, hwaddr addr,
                                      uint8_t *buf, int len)
{
    pci_dma_write(dma_opaque, addr, buf, len);
}

static void pci_physical_memory_read(void *dma_opaque, hwaddr addr,
                                     uint8_t *buf, int len)
{
   pci_dma_read(dma_opaque, addr, buf, len);
}

static void pci_tulip_cleanup(NetClientState *nc)
{
    TulipState *s = qemu_get_nic_opaque(nc);

    tulip_cleanup(s);
}

static void pci_tulip_uninit(PCIDevice *dev)
{
    PCITulipState *d = PCI_TULIP(dev);

    qemu_free_irq(d->state.irq);
    memory_region_destroy(&d->state.mmio);
    memory_region_destroy(&d->io_bar);
    timer_del(d->state.timer);
    timer_free(d->state.timer);
    eeprom93xx_free(&dev->qdev, d->state.eeprom);
    qemu_del_nic(d->state.nic);
}

static NetClientInfo net_tulip_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .can_receive = tulip_can_receive,
    .receive = tulip_receive,
    .cleanup = pci_tulip_cleanup,
    .link_status_changed = tulip_set_link_status,
};

static int pci_tulip_init(PCIDevice *pci_dev)
{
    PCITulipState *d = PCI_TULIP(pci_dev);
    TulipState *s = &d->state;
    uint8_t *pci_conf;

    pci_conf = pci_dev->config;

    /* TODO: RST# value should be 0, PCI spec 6.2.4 */
    pci_conf[PCI_CACHE_LINE_SIZE] = 0x10;

    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    /* PCI interface */
    memory_region_init_io(&d->state.mmio, OBJECT(d), &tulip_mmio_ops, s,
                          "tulip-mmio", TULIP_CSR_REGION_SIZE);
    memory_region_init_io(&d->io_bar, OBJECT(d), &tulip_mmio_ops, s,
                          "tulip-io", TULIP_CSR_REGION_SIZE);

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->io_bar);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->state.mmio);

    s->irq = pci_allocate_irq(pci_dev);

    s->phys_mem_read = pci_physical_memory_read;
    s->phys_mem_write = pci_physical_memory_write;
    s->dma_opaque = pci_dev;

    /* FIXME: Move everything below to this func: */
    return tulip_init(DEVICE(pci_dev), s, &net_tulip_info);
}

static void pci_tulip_reset(DeviceState *dev)
{
    PCITulipState *d = PCI_TULIP(dev);

    tulip_reset(&d->state);
}

static Property tulip_properties[] = {
    DEFINE_NIC_PROPERTIES(PCITulipState, state.conf),
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
    dc->reset = pci_tulip_reset;
    dc->vmsd = &vmstate_tulip;
    dc->props = tulip_properties;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo tulip_info = {
    .name          = TYPE_PCI_TULIP,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCITulipState),
    .class_init    = tulip_class_init,
};

static void tulip_register_types(void)
{
    type_register_static(&tulip_info);
}

type_init(tulip_register_types)

