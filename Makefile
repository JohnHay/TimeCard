
LOCALBASE?=	/usr/local
PVER:sh=	cat version
WORKDIR=	${.CURDIR}/work
STAGEDIR=	${WORKDIR}/staging

SUBDIR=	\
	modules \
	tc-adj-servo \
	tc-fw-flash \
	timecard

INCFILES=	\
	modules/timecard/timecard.h	\
	modules/timecard/timecard_bus.h	\
	modules/timecard/timecard_reg.h
INCDEST=	${LOCALBASE}/include

MAN4FILES=	\
	modules/timecard/timecard.4			\
	modules/axi_iic/axi_iic.4			\
	modules/axi_spi_timecard/axi_spi_timecard.4	\
	modules/uart_timecard/uart_timecard.4
MAN4DEST=	${LOCALBASE}/man/man4
MAN8DEST=	${LOCALBASE}/man/man8

package:
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${KMODDIR}
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${LOCALBASE}/sbin
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${MAN4DEST}
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${MAN8DEST}
	make
	make DESTDIR=${STAGEDIR} install
	make DESTDIR=${STAGEDIR} installconfig
	make DESTDIR=${STAGEDIR} include-package
	make DESTDIR=${STAGEDIR} man-package
	make WORKDIR=${WORKDIR} version-package
	${PKG_CMD} create -v -r ${STAGEDIR} -M ${WORKDIR}/pkg-manifest -p pkg-plist

clean-package:
	rm -rf ${WORKDIR}

include-package:
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${DESTDIR}${LOCALBASE}/include
	install -m 444 -o ${BINOWN} -g ${BINGRP} ${INCFILES} ${DESTDIR}${INCDEST}/

man-package:
	install -m 444 -o ${BINOWN} -g ${BINGRP} ${MAN4FILES} ${DESTDIR}${MAN4DEST}/
	gzip -f ${DESTDIR}${MAN4DEST}/*.4

version-package:
	@echo version ${PVER}
	@sed -e "s/%VERSION%/${PVER}/" < pkg-manifest > ${WORKDIR}/pkg-manifest

.include <bsd.subdir.mk>
