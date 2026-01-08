
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
RCDMODE=	555

USE_BME=	yes
.if USE_BME
BME_DIR?=../../BME280_SensorAPI-master
SRCS+=	${BME_DIR}/bme280.c
CFLAGS+=	-I ${BME_DIR} -DUSE_BME
.endif

.include <bsd.prog.mk>
