/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 John Hay
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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include "device_if.h"
#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include "axi_quad_spi.h"
#include "spibus_if.h"

#include <timecard_bus.h>

static int
timecard_spi_probe(device_t dev)
{
	device_t parent;
	uintptr_t type;

	parent = device_get_parent(dev);

	if (BUS_READ_IVAR(parent, dev, TIMECARD_IVAR_TYPE, &type))
		return (ENXIO);
	if (type != TIMECARD_TYPE_AXI_QSPI)
		return (ENXIO);

	if (device_get_desc(dev) == NULL)
		device_set_desc(dev, "Xilinx Quad SPI");
	return axi_spi_probe(dev);
}


static device_method_t timecard_spi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		timecard_spi_probe),
	DEVMETHOD(device_attach,	axi_spi_attach),
	DEVMETHOD(device_detach,	axi_spi_detach),

	/* SPI interface */
	DEVMETHOD(spibus_transfer,	axi_spi_transfer),
	DEVMETHOD_END
};

static driver_t timecard_spi_driver = {
	"spi",
	timecard_spi_methods,
	sizeof(struct axi_spi_softc),
};

DRIVER_MODULE(spi, timecard, timecard_spi_driver, 0, 0);
MODULE_DEPEND(spi, spibus, 1, 1, 1);
