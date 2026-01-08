/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 John Hay
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/kobj.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <dev/iicbus/iic.h>
#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <machine/bus.h>
#include "iicbus_if.h"

#include <timecard_bus.h>

#define AXI_IIC_GIE_REG		0x01C
#define IIC_GIE_ENA		0x01
#define AXI_IIC_ISR_REG		0x020
#define IIC_ISR_ARB_LOST	(1 << 0)
#define IIC_ISR_TX_ERROR	(1 << 1)
#define IIC_ISR_TX_EMPTY	(1 << 2)
#define IIC_ISR_RX_FULL		(1 << 3)
#define IIC_ISR_BUS_IDLE	(1 << 4)
#define IIC_ISR_ADR_SLAVE	(1 << 5)
#define IIC_ISR_NOT_ADDR_SLAVE	(1 << 6)
#define IIC_ISR_TX_HALF_EMPTY	(1 << 7)

#define AXI_IIC_IER_REG		0x028
/* Use same as ISR */
#define AXI_IIC_SRR_REG		0x040
#define IIC_SRR_RKEY		0x0A

#define AXI_IIC_CR_REG		0x100
#define IIC_CR_GC_EN		(1 << 6)
#define IIC_CR_RSTA		(1 << 5)
#define IIC_CR_TXAK		(1 << 4)
#define IIC_CR_TX		(1 << 3)
#define IIC_CR_MSMS		(1 << 2)
#define IIC_CR_TX_FIFO_RESET	(1 << 1)
#define IIC_CR_EN		(1 << 0)

#define AXI_IIC_SR_REG		0x104
#define IIC_SR_TX_FIFO_EMPTY	(1 << 7)
#define IIC_SR_RX_FIFO_EMPTY	(1 << 6)
#define IIC_SR_RX_FIFO_FULL	(1 << 5)
#define IIC_SR_TX_FIFO_FULL	(1 << 4)
#define IIC_SR_SRW		(1 << 3)
#define IIC_SR_BB		(1 << 2)
#define IIC_SR_AAS		(1 << 1)
#define IIC_SR_ABGC		(1 << 0)

#define AXI_IIC_TX_FIFO_REG	0x108
#define IIC_TX_STOP		(1 << 9)
#define IIC_TX_START		(1 << 8)
#define IIC_TX_RD		(1 << 0)

#define AXI_IIC_RX_FIFO_REG	0x10C
#define AXI_IIC_ADR_REG		0x110
#define AXI_IIC_TENADR_REG	0x11C
#define AXI_IIC_TXOCY_REG	0x114
#define IIC_TXOCY_MASK		0x0F
#define AXI_IIC_RXOCY_REG	0x118
#define IIC_RXOCY_MASK		0x0F
#define AXI_IIC_RXPIRQ_REG	0x120
#define AXI_IIC_GPO_REG		0x124
#define AXI_IIC_TSUSTA_REG	0x128
#define AXI_IIC_TSUSTO_REG	0x12C
#define AXI_IIC_THDSTA_REG	0x130
#define AXI_IIC_TSUDAT_REG	0x134
#define AXI_IIC_TBUF_REG	0x138
#define AXI_IIC_THIGH_REG	0x13C
#define AXI_IIC_TLOW_REG	0x140
#define AXI_IIC_THDDAT_REG	0x144

struct axi_iic_softc {
	device_t sc_dev;
	device_t iicbus;
	device_t iic;
	struct resource *sc_res;
	int sc_res_type;
	int sc_rid;
	int sc_lowto;
	uint32_t iicerrcnt;
};

static int xfer_verbose;
SYSCTL_INT(_debug, OID_AUTO, xfer_verbose, CTLFLAG_RW | CTLFLAG_MPSAFE,
    &xfer_verbose, 0, "Enable verbose output in axi_iic_xfer()");

static int axi_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr);

static int
axi_iic_probe(device_t dev)
{
	device_t parent;
	uintptr_t type;

	parent = device_get_parent(dev);

	if (BUS_READ_IVAR(parent, dev, TIMECARD_IVAR_TYPE, &type))
		return (ENXIO);
	if (type != TIMECARD_TYPE_AXI_IIC)
		return (ENXIO);

	if (device_get_desc(dev) == NULL)
		device_set_desc(dev, "Xilinx AXI IIC");

	return (BUS_PROBE_DEFAULT);
}

static int
axi_iic_attach(device_t dev)
{
	struct axi_iic_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	sc->sc_rid = 0;
	sc->sc_res_type = SYS_RES_MEMORY;
	sc->sc_res = bus_alloc_resource_any(dev, sc->sc_res_type, &sc->sc_rid, RF_ACTIVE);
	if (sc->sc_res == NULL)
		return (ENXIO);
	device_printf(dev, "axi_iic_attach: res allocated %lX\n", rman_get_start(sc->sc_res));

	sc->sc_lowto = 1000000;
	/* Soft reset */
	bus_write_4(sc->sc_res, AXI_IIC_SRR_REG, IIC_SRR_RKEY);
	/* max rx fifo, clear tx fifo and enable iic */
	bus_write_4(sc->sc_res, AXI_IIC_RXPIRQ_REG, IIC_RXOCY_MASK);
	bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN | IIC_CR_TX_FIFO_RESET);
	bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);

	if (bus_read_4(sc->sc_res, AXI_IIC_CR_REG) != IIC_CR_EN) {
		device_printf(dev, "attach() unexpected values CR %X, SR %X\n",
		    bus_read_4(sc->sc_res, AXI_IIC_CR_REG),
		    bus_read_4(sc->sc_res, AXI_IIC_SR_REG));
		/* XXX Should probably fail here. */
		bus_release_resource(dev, sc->sc_res_type, sc->sc_rid, sc->sc_res);
		sc->sc_res = NULL;
		return (ENXIO);
	}

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "iicbus creation failed\n");
		bus_release_resource(dev, sc->sc_res_type, sc->sc_rid, sc->sc_res);
		sc->sc_res = NULL;
		return (ENXIO);
	}
	bus_attach_children(dev);
	return (0);
}

static int
axi_iic_detach(device_t dev)
{
	struct axi_iic_softc *sc;

	sc = device_get_softc(dev);
	/* If the child cannot be deleted, we should not release the resources */
	if (sc->iic != NULL) {
		if (device_delete_child(dev, sc->iic) == 0)
			sc->iic = NULL;
	}
	if (sc->iicbus != NULL) {
		if (device_delete_child(dev, sc->iicbus) == 0)
			sc->iicbus = NULL;
		else
			return (ENXIO);
	}
	if (sc->sc_res != NULL) {
		bus_release_resource(dev, sc->sc_res_type, sc->sc_rid, sc->sc_res);
		sc->sc_res = NULL;
	}
	return (0);
}

static int
axi_iic_xfer(device_t dev, struct iic_msg *msgs, uint32_t num)
{
	struct axi_iic_softc *sc;
	bool last, reading;
	int err = 0, to;
	uint8_t *buf;
	uint16_t addr;
	uint32_t b, len, m, psr, sr;

	sc = device_get_softc(dev);

#if 0
	printf("axi_iic_xfer() num msgs %u\n", num);
#endif

	/* If the TX fifo is not empty when we get here, something must have
	 * gone wrong previously, so reset it.
	 */
	if ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_EMPTY) == 0) {
		bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN | IIC_CR_TX_FIFO_RESET);
		bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
	}
#if 0
	/* Wait for fifo to empty. Clear TX fifo */
	while ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_EMPTY) == 0)
		;
#endif
	/* Clear RX fifo */
	while ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_RX_FIFO_EMPTY) == 0)
		bus_read_4(sc->sc_res, AXI_IIC_RX_FIFO_REG);
	/* Wait if Bus Busy */
	while ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_BB) != 0)
		;
	bus_write_4(sc->sc_res, AXI_IIC_CR_REG, 0);

	for (m = 0; m < num; m++) {

		addr = msgs[m].slave;
		len = msgs[m].len;
		buf = msgs[m].buf;
		reading = (msgs[m].flags & IIC_M_RD) != 0;
		last = m == (num - 1);
		bus_write_4(sc->sc_res, AXI_IIC_CR_REG, 0);

#if 0
		printf("axi_iic_xfer() msg %u, len %u, reading %u, last %u, addr %X, buf[0] %X\n",
		    m, len, reading, last, addr, buf[0]);
#endif
		if (reading) {
			bus_write_4(sc->sc_res, AXI_IIC_TX_FIFO_REG, addr | IIC_TX_START | IIC_TX_RD);
			bus_write_4(sc->sc_res, AXI_IIC_TX_FIFO_REG, len | IIC_TX_STOP);
			bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
			psr = sr = 0;
			for (b = 0; b < len; b++) {
				to = 100000;
				for (;;) {
					sr = bus_read_4(sc->sc_res, AXI_IIC_SR_REG);
					if (xfer_verbose && (sr != psr))
						printf("axi rd pSR %X, SR %X, to %d, b %u\n", psr, sr, to, b);
					psr = sr;
					if ((sr & IIC_SR_RX_FIFO_EMPTY) == 0)
						break;
					/* If the RX FIFO is empty and the bus not busy, something went wrong. */
					if ((sr & IIC_SR_BB) == 0) {
						sc->iicerrcnt++;
						if ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_EMPTY) == 0) {
							bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN | IIC_CR_TX_FIFO_RESET);
							bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
						}
						return (IIC_ENOACK);
					}
					to--;
					if (to <= 0) {
						sc->iicerrcnt++;
						device_printf(dev,
						    "axi_iic_xfer() read timeout CR %X, SR %X\n",
						    bus_read_4(sc->sc_res, AXI_IIC_CR_REG),
						    bus_read_4(sc->sc_res, AXI_IIC_SR_REG));
						axi_iic_reset(dev, IIC_UNKNOWN, addr, NULL);
						return (IIC_ETIMEOUT);
					}
				}
#if 0
				while (bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_RX_FIFO_EMPTY) {
					to--;
					if (to <= 0) {
						sc->iicerrcnt++;
						device_printf(dev,
						    "axi_iic_xfer() read timeout CR %X, SR %X\n",
						    bus_read_4(sc->sc_res, AXI_IIC_CR_REG),
						    bus_read_4(sc->sc_res, AXI_IIC_SR_REG));
						axi_iic_reset(dev, IIC_UNKNOWN, addr, NULL);
						return (IIC_ETIMEOUT);
					}
				}
#endif
				buf[b] = bus_read_4(sc->sc_res, AXI_IIC_RX_FIFO_REG);
				if ((to > 0) && (to < sc->sc_lowto)) {
					device_printf(dev, "lowto %d\n", to);
					sc->sc_lowto = to;
				}
			}
		} else {
			bus_write_4(sc->sc_res, AXI_IIC_TX_FIFO_REG, addr | IIC_TX_START);
			for (b = 0; b < len; b++) {
				while (bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_FULL)
					if ((bus_read_4(sc->sc_res, AXI_IIC_CR_REG) & IIC_CR_EN) == 0)
						bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
				if (last && ((b + 1) == len))
					bus_write_4(sc->sc_res, AXI_IIC_TX_FIFO_REG, buf[b] | IIC_TX_STOP);
				else
					bus_write_4(sc->sc_res, AXI_IIC_TX_FIFO_REG, buf[b]);
			}
			if (last) {
				if ((bus_read_4(sc->sc_res, AXI_IIC_CR_REG) & IIC_CR_EN) == 0)
					bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
				for (;;) {
					sr = bus_read_4(sc->sc_res, AXI_IIC_SR_REG);
					if ((sr & IIC_SR_TX_FIFO_EMPTY) == IIC_SR_TX_FIFO_EMPTY)
						break;
					/* If the TX FIFO is not empty and the bus not busy, something went wrong. */
					if ((sr & IIC_SR_BB) == 0) {
						sc->iicerrcnt++;
						if ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_EMPTY) == 0) {
							bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN | IIC_CR_TX_FIFO_RESET);
							bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);
						}
						return (IIC_ENOACK);
					}
					to--;
				}
#if 0
				while ((bus_read_4(sc->sc_res, AXI_IIC_SR_REG) & IIC_SR_TX_FIFO_EMPTY) == 0)
					;
#endif
			}
		}

		if (err)
			break;
	}
	return (err);
}

static int
axi_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct axi_iic_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_lowto = 1000000;
	/* Soft reset */
	bus_write_4(sc->sc_res, AXI_IIC_SRR_REG, IIC_SRR_RKEY);
	/* max rx fifo, clear tx fifo and enable iic */
	bus_write_4(sc->sc_res, AXI_IIC_RXPIRQ_REG, IIC_RXOCY_MASK);
	bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN | IIC_CR_TX_FIFO_RESET);
	bus_write_4(sc->sc_res, AXI_IIC_CR_REG, IIC_CR_EN);

	return (0);
}

static device_method_t axi_iic_methods[] = {
	DEVMETHOD(device_probe,		axi_iic_probe),
	DEVMETHOD(device_attach,	axi_iic_attach),
	DEVMETHOD(device_detach,	axi_iic_detach),

	DEVMETHOD(iicbus_reset,		axi_iic_reset),
	DEVMETHOD(iicbus_transfer,	axi_iic_xfer),
	DEVMETHOD(iicbus_callback, iicbus_null_callback),
	DEVMETHOD_END
};

static driver_t axi_iic_driver = {
	"axi_iic",
	axi_iic_methods,
	sizeof(struct axi_iic_softc)
};

DRIVER_MODULE(axi_iic, timecard, axi_iic_driver, 0, 0);
MODULE_DEPEND(axi_iic, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(axi_iic, 1);

DRIVER_MODULE(iicbus, axi_iic, iicbus_driver, 0, 0);
