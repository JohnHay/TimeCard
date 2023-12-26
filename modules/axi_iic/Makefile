# Makefile for axi_iic driver

KMOD=	axi_iic
SRCS=	axi_iic.c
SRCS+=	bus_if.h device_if.h iicbus_if.h

CFLAGS+= -I../timecard

.include <bsd.kmod.mk>
