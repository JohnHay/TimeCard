
# Use realinstallconfig to install config files

PROG=	timecard
MAN=	timecard.8
SRCS=	${PROG}.c
LDADD+= -ldevinfo
CFLAGS+= -I../modules/timecard

CONFGROUPS=	CONFLOG RCD
CONFLOGDIR=	/usr/local/etc/syslog.d
CONFLOG=	timecard.conf
CONGLOGMODE=	644

RCD=		timecard.sh
RCDNAME=	timecard
RCDDIR=		/usr/local/etc/rc.d

.if USE_BME
SRCS+=	BME280_driver-master/bme280.c
CFLAGS+=	-I BME280_driver-master
.endif

.include <bsd.prog.mk>
