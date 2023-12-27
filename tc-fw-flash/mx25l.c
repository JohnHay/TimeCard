/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2009 Oleksandr Tymoshenko.  All rights reserved.
 * Copyright (c) 2018 Ian Lepore.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh <imp@FreeBSD.org>
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

#include "mx25lreg.h"

#define	FL_NONE			0x00
#define	FL_ERASE_4K		0x01
#define	FL_ERASE_32K		0x02
#define	FL_ENABLE_4B_ADDR	0x04
#define	FL_DISABLE_4B_ADDR	0x08

/*
 * Define the sectorsize to be a smaller size rather than the flash
 * sector size. Trying to run FFS off of a 64k flash sector size
 * results in a completely un-usable system.
 */
#define	MX25L_SECTORSIZE	512

struct mx25l_flash_ident
{
	const char	*name;
	uint8_t		manufacturer_id;
	uint16_t	device_id;
	unsigned int	sectorsize;
	unsigned int	sectorcount;
	unsigned int	flags;
};

struct mx25l_softc 
{
	device_t	sc_parent;
	uint8_t		sc_manufacturer_id;
	uint16_t	sc_device_id;
	unsigned int	sc_erasesize;
	unsigned int	sc_flags;
	uint8_t		sc_dummybuf[FLASH_PAGE_SIZE];
};

#define	TSTATE_STOPPED	0
#define	TSTATE_STOPPING	1
#define	TSTATE_RUNNING	2

static struct mx25l_flash_ident flash_devices[] = {
	{ "en25f32",	0x1c, 0x3116, 64 * 1024, 64, FL_NONE },
	{ "en25p32",	0x1c, 0x2016, 64 * 1024, 64, FL_NONE },
	{ "en25p64",	0x1c, 0x2017, 64 * 1024, 128, FL_NONE },
	{ "en25q32",	0x1c, 0x3016, 64 * 1024, 64, FL_NONE },
	{ "en25q64",	0x1c, 0x3017, 64 * 1024, 128, FL_ERASE_4K },
	{ "m25p32",	0x20, 0x2016, 64 * 1024, 64, FL_NONE },
	{ "m25p64",	0x20, 0x2017, 64 * 1024, 128, FL_NONE },
	{ "mt25ql128",	0x20, 0xba18, 64 * 1024, 256, FL_ERASE_4K | FL_ERASE_32K },
	{ "mx25l1606e", 0xc2, 0x2015, 64 * 1024, 32, FL_ERASE_4K},
	{ "mx25ll32",	0xc2, 0x2016, 64 * 1024, 64, FL_NONE },
	{ "mx25ll64",	0xc2, 0x2017, 64 * 1024, 128, FL_NONE },
	{ "mx25ll128",	0xc2, 0x2018, 64 * 1024, 256, FL_ERASE_4K | FL_ERASE_32K },
	{ "mx25ll256",	0xc2, 0x2019, 64 * 1024, 512, FL_ERASE_4K | FL_ERASE_32K | FL_ENABLE_4B_ADDR },
	{ "n25q64",	0x20, 0xba17, 64 * 1024, 128, FL_ERASE_4K },
	{ "s25fl032",	0x01, 0x0215, 64 * 1024, 64, FL_NONE },
	{ "s25fl064",	0x01, 0x0216, 64 * 1024, 128, FL_NONE },
	{ "s25fl128",	0x01, 0x2018, 64 * 1024, 256, FL_NONE },
	{ "s25fl256s",	0x01, 0x0219, 64 * 1024, 512, FL_NONE },
	{ "s25fl512s",	0x01, 0x0220, 64 * 1024, 1024, FL_NONE },
	{ "SST25VF010A", 0xbf, 0x2549, 4 * 1024, 32, FL_ERASE_4K | FL_ERASE_32K },
	{ "SST25VF032B", 0xbf, 0x254a, 64 * 1024, 64, FL_ERASE_4K | FL_ERASE_32K },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x32",	0xef, 0x3016, 64 * 1024, 64, FL_ERASE_4K },
	{ "w25x64",	0xef, 0x3017, 64 * 1024, 128, FL_ERASE_4K },
	{ "w25q32",	0xef, 0x4016, 64 * 1024, 64, FL_ERASE_4K },
	{ "w25q64",	0xef, 0x4017, 64 * 1024, 128, FL_ERASE_4K },
	{ "w25q64bv",	0xef, 0x4017, 64 * 1024, 128, FL_ERASE_4K },
	{ "w25q128",	0xef, 0x4018, 64 * 1024, 256, FL_ERASE_4K },
	{ "w25q256",	0xef, 0x4019, 64 * 1024, 512, FL_ERASE_4K },

	 /* Atmel */
	{ "at25df641",  0x1f, 0x4800, 64 * 1024, 128, FL_ERASE_4K },

	/* GigaDevice */
	{ "gd25q64",	0xc8, 0x4017, 64 * 1024, 128, FL_ERASE_4K },
	{ "gd25q128",	0xc8, 0x4018, 64 * 1024, 256, FL_ERASE_4K },

	/* Integrated Silicon Solution */
	{ "is25wp256",	0x9d, 0x7019, 64 * 1024, 512, FL_ERASE_4K | FL_ENABLE_4B_ADDR},
};

static int
mx25l_wait_for_device_ready(struct mx25l_softc *sc)
{
	uint8_t txBuf[2], rxBuf[2];
	struct spi_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));

	do {
		txBuf[0] = CMD_READ_STATUS;
		cmd.tx_cmd = txBuf;
		cmd.rx_cmd = rxBuf;
		cmd.rx_cmd_sz = 2;
		cmd.tx_cmd_sz = 2;
		err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd);
	} while (err == 0 && (rxBuf[1] & STATUS_WIP));
	return (err);
}

static struct mx25l_flash_ident*
mx25l_get_device_ident(struct mx25l_softc *sc)
{
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	uint8_t manufacturer_id;
	uint16_t dev_id;
	int err, i;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = CMD_READ_IDENT;
	cmd.tx_cmd = &txBuf;
	cmd.rx_cmd = &rxBuf;
	/*
	 * Some compatible devices has extended two-bytes ID
	 * We'll use only manufacturer/deviceid atm
	 */
	cmd.tx_cmd_sz = 4;
	cmd.rx_cmd_sz = 4;
	err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd);
	if (err)
		return (NULL);

	manufacturer_id = rxBuf[1];
	dev_id = (rxBuf[2] << 8) | (rxBuf[3]);

	for (i = 0; i < nitems(flash_devices); i++) {
		if ((flash_devices[i].manufacturer_id == manufacturer_id) &&
		    (flash_devices[i].device_id == dev_id))
			return &flash_devices[i];
	}

	printf("Unknown SPI flash device. Vendor: %02x, device id: %04x\n",
	    manufacturer_id, dev_id);
	return (NULL);
}

static int
mx25l_set_writable(struct mx25l_softc *sc, int writable)
{
	uint8_t txBuf[1], rxBuf[1];
	struct spi_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = writable ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;
	cmd.tx_cmd = txBuf;
	cmd.rx_cmd = rxBuf;
	cmd.rx_cmd_sz = 1;
	cmd.tx_cmd_sz = 1;
	err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd);
	return (err);
}

static int
mx25l_erase_cmd(struct mx25l_softc *sc, off_t sector)
{
	uint8_t txBuf[5], rxBuf[5];
	struct spi_command cmd;
	int err;

	if ((err = mx25l_set_writable(sc, 1)) != 0)
		return (err);

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	cmd.tx_cmd = txBuf;
	cmd.rx_cmd = rxBuf;

	if (sc->sc_flags & FL_ERASE_4K)
		txBuf[0] = CMD_BLOCK_4K_ERASE;
	else if (sc->sc_flags & FL_ERASE_32K)
		txBuf[0] = CMD_BLOCK_32K_ERASE;
	else
		txBuf[0] = CMD_SECTOR_ERASE;

	if (sc->sc_flags & FL_ENABLE_4B_ADDR) {
		cmd.rx_cmd_sz = 5;
		cmd.tx_cmd_sz = 5;
		txBuf[1] = ((sector >> 24) & 0xff);
		txBuf[2] = ((sector >> 16) & 0xff);
		txBuf[3] = ((sector >> 8) & 0xff);
		txBuf[4] = (sector & 0xff);
	} else {
		cmd.rx_cmd_sz = 4;
		cmd.tx_cmd_sz = 4;
		txBuf[1] = ((sector >> 16) & 0xff);
		txBuf[2] = ((sector >> 8) & 0xff);
		txBuf[3] = (sector & 0xff);
	}
	if ((err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd)) != 0)
		return (err);
	err = mx25l_wait_for_device_ready(sc);
	return (err);
}

static int
mx25l_write(struct mx25l_softc *sc, off_t offset, caddr_t data, off_t count, int verbose)
{
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	off_t bytes_to_write;
	int err = 0;
	int preverb = 0;

	if (sc->sc_flags & FL_ENABLE_4B_ADDR) {
		cmd.tx_cmd_sz = 5;
		cmd.rx_cmd_sz = 5;
	} else {
		cmd.tx_cmd_sz = 4;
		cmd.rx_cmd_sz = 4;
	}

	/*
	 * Writes must be aligned to the erase sectorsize, since blocks are
	 * fully erased before they're written to.
	 */
	if (offset % sc->sc_erasesize != 0)
		return (EIO);

	/*
	 * Maximum write size for CMD_PAGE_PROGRAM is FLASH_PAGE_SIZE, so loop
	 * to write chunks of FLASH_PAGE_SIZE bytes each.
	 */
	while (count != 0) {
		/* If we crossed a sector boundary, erase the next sector. */
		if (((offset) % sc->sc_erasesize) == 0) {
			err = mx25l_erase_cmd(sc, offset);
			if (err)
				break;
		}
		if (verbose && ((count / verbose) != preverb)) {
			printf(".");
			fflush(stdout);
			preverb = count / verbose;
		}

		txBuf[0] = CMD_PAGE_PROGRAM;
		if (sc->sc_flags & FL_ENABLE_4B_ADDR) {
			txBuf[1] = (offset >> 24) & 0xff;
			txBuf[2] = (offset >> 16) & 0xff;
			txBuf[3] = (offset >> 8) & 0xff;
			txBuf[4] = offset & 0xff;
		} else {
			txBuf[1] = (offset >> 16) & 0xff;
			txBuf[2] = (offset >> 8) & 0xff;
			txBuf[3] = offset & 0xff;
		}

		bytes_to_write = MIN(FLASH_PAGE_SIZE, count);
		cmd.tx_cmd = txBuf;
		cmd.rx_cmd = rxBuf;
		cmd.tx_data = data;
		cmd.rx_data = sc->sc_dummybuf;
		cmd.tx_data_sz = (uint32_t)bytes_to_write;
		cmd.rx_data_sz = (uint32_t)bytes_to_write;

		/*
		 * Each completed write operation resets WEL (write enable
		 * latch) to disabled state, so we re-enable it here.
		 */
		if ((err = mx25l_wait_for_device_ready(sc)) != 0)
			break;
		if ((err = mx25l_set_writable(sc, 1)) != 0)
			break;

		err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd);
		if (err != 0)
			break;
		err = mx25l_wait_for_device_ready(sc);
		if (err)
			break;

		data   += bytes_to_write;
		offset += bytes_to_write;
		count  -= bytes_to_write;
	}

	return (err);
}

static int
mx25l_read(struct mx25l_softc *sc, off_t offset, caddr_t data, off_t count)
{
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	int err = 0;

	txBuf[0] = CMD_FAST_READ;
	if (sc->sc_flags & FL_ENABLE_4B_ADDR) {
		cmd.tx_cmd_sz = 6;
		cmd.rx_cmd_sz = 6;

		txBuf[1] = (offset >> 24) & 0xff;
		txBuf[2] = (offset >> 16) & 0xff;
		txBuf[3] = (offset >> 8) & 0xff;
		txBuf[4] = offset & 0xff;
		/* Dummy byte */
		txBuf[5] = 0;
	} else {
		cmd.tx_cmd_sz = 5;
		cmd.rx_cmd_sz = 5;

		txBuf[1] = (offset >> 16) & 0xff;
		txBuf[2] = (offset >> 8) & 0xff;
		txBuf[3] = offset & 0xff;
		/* Dummy byte */
		txBuf[4] = 0;
	}

	cmd.tx_cmd = txBuf;
	cmd.rx_cmd = rxBuf;
	cmd.tx_data = data;
	cmd.rx_data = data;
	cmd.tx_data_sz = count;
	cmd.rx_data_sz = count;

	err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd);
	return (err);
}

static int
mx25l_set_4b_mode(struct mx25l_softc *sc, uint8_t command)
{
	uint8_t txBuf[1], rxBuf[1];
	struct spi_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	cmd.tx_cmd_sz = cmd.rx_cmd_sz = 1;

	cmd.tx_cmd = txBuf;
	cmd.rx_cmd = rxBuf;

	txBuf[0] = command;

	if ((err = SPIBUS_TRANSFER(sc->sc_parent, sc->sc_dev, &cmd)) == 0)
		err = mx25l_wait_for_device_ready(sc);

	return (err);
}

static int
mx25l_init(struct mx25l_softc *sc)
{
	struct mx25l_flash_ident *ident;
	int err;

	ident = mx25l_get_device_ident(sc);
	if (ident == NULL)
		return (ENXIO);

	if ((err = mx25l_wait_for_device_ready(sc)) != 0)
		return (err);

	sc->sc_flags = ident->flags;

	if (sc->sc_flags & FL_ERASE_4K)
		sc->sc_erasesize = 4 * 1024;
	else if (sc->sc_flags & FL_ERASE_32K)
		sc->sc_erasesize = 32 * 1024;
	else
		sc->sc_erasesize = ident->sectorsize;

	if (sc->sc_flags & FL_ENABLE_4B_ADDR) {
		if ((err = mx25l_set_4b_mode(sc, CMD_ENTER_4B_MODE)) != 0)
			return (err);
	} else if (sc->sc_flags & FL_DISABLE_4B_ADDR) {
		if ((err = mx25l_set_4b_mode(sc, CMD_EXIT_4B_MODE)) != 0)
			return (err);
	}

	printf("device type %s, size %dK in %d sectors of %dK, erase size %dK\n",
	    ident->name,
	    ident->sectorcount * ident->sectorsize / 1024,
	    ident->sectorcount, ident->sectorsize / 1024,
	    sc->sc_erasesize / 1024);

	return (0);
}

