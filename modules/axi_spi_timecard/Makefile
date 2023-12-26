# Makefile for axi_qspi_timecard driver

#.PATH:	${SRCTOP}/sys/dev/xilinx

KMOD=	axi_spi_timecard
SRCS=	axi_spi_timecard.c
SRCS+=	axi_quad_spi.c
SRCS+=	bus_if.h device_if.h opt_platform.h spibus_if.h

.if !empty(OPT_FDT)
SRCS+=  ofw_bus_if.h
.endif

CFLAGS+= -I../timecard

.include <bsd.kmod.mk>
