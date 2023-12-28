
LOCALBASE?=	/usr/local
WORKDIR=	${.CURDIR}/work
STAGEDIR=	${WORKDIR}/staging

SUBDIR=	\
	modules \
	tc-fw-flash \
	timecard

package:
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${KMODDIR}
	install -d -m ${DIRMODE} -o ${DIROWN} -g ${DIRGRP} ${STAGEDIR}${LOCALBASE}/sbin
	make
	make DESTDIR=${STAGEDIR} install
	make DESTDIR=${STAGEDIR} installconfig
	${PKG_CMD} create -v -r ${STAGEDIR} -M pkg-manifest -p pkg-plist

clean-package:
	rm -rf ${WORKDIR}

.include <bsd.subdir.mk>
