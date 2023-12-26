/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <sys/serial.h>
#include <serdev_if.h>
#include <timecard_bus.h>

#define AXI_UART_RWIDT		4
#define AXI_UART_SHIFT		2

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>

static int uart_timecard_probe(device_t dev);

static device_method_t uart_timecard_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		uart_timecard_probe),
	DEVMETHOD(device_attach,	uart_bus_attach),
	DEVMETHOD(device_detach,	uart_bus_detach),
	/* Serdev interface */
	DEVMETHOD(serdev_ihand,		uart_bus_ihand),
	DEVMETHOD(serdev_ipend,		uart_bus_ipend),
	{ 0, 0 }
};

static driver_t uart_timecard_driver = {
	uart_driver_name,
	uart_timecard_methods,
	sizeof(struct uart_softc),
};

static int
uart_timecard_probe(device_t dev)
{
	device_t parent;
	struct uart_softc *sc;
	uintptr_t rclk, type;

	parent = device_get_parent(dev);
	sc = device_get_softc(dev);

	if (BUS_READ_IVAR(parent, dev, TIMECARD_IVAR_TYPE, &type))
		return (ENXIO);
	if (type != TIMECARD_TYPE_SERIAL)
		return (ENXIO);

	sc->sc_class = &uart_ns8250_class;

	if (BUS_READ_IVAR(parent, dev, TIMECARD_IVAR_CLOCK, &rclk))
		rclk = 0;
	return (uart_bus_probe(dev, AXI_UART_SHIFT, AXI_UART_RWIDT, rclk,
	    0, 0, 0));
}

DRIVER_MODULE(uart, timecard, uart_timecard_driver, 0, 0);
