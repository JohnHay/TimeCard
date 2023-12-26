
PACKAGE=timecard
PROG=	timecard
#MAN=	timecard.8
MAN=
SRCS=	${PROG}.c
LDADD+= -ldevinfo
CFLAGS+= -I../modules/timecard

.if USE_BME
SRCS+=	BME280_driver-master/bme280.c
CFLAGS+=	-I BME280_driver-master
.endif

BINDIR?=        /usr/local/sbin
MANDIR?=	/usr/local/man

.include <bsd.prog.mk>
