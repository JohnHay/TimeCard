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

/*
 * Program the FPGA firmware flash on TimeCards.
 * Programming can happen in two ways:
 * - Use /dev/spigen0.0 if available.
 * - Memory map the Timecard and write to the Flash on the FPGA through the
 *   AXI QSPI port directly. This is meant to be used when bringing up a
 *   card and the driver is not loaded yet.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <termios.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/spigenio.h>

#define DEFAULTDEV	"/dev/spigen0.0"

struct spi_command {
	void    *tx_cmd;
	uint32_t tx_cmd_sz;
	void    *rx_cmd;
	uint32_t rx_cmd_sz;
	void    *tx_data;
	uint32_t tx_data_sz;
	void    *rx_data;
	uint32_t rx_data_sz;
};

struct spi_softc {
	void *base;
	int spidev;
};

struct mx25l_softc;

static int read_flash(struct mx25l_softc *sc, uint32_t fl_offset, char *fname);
static int write_flash(struct mx25l_softc *sc, uint32_t fl_offset, int verify, char *fname);
static void printaxiregs(struct spi_softc *sc);
static int pcie_map(off_t target);
static int axi_init(struct spi_softc *sc);
static int spi_transfer(struct spi_softc *sc, struct spi_command *cmd);

#define DELAY usleep

#define	SPIBUS_CS_HIGH	(1U << 31)

#define	READ4(_sc, _reg)	\
	*(volatile uint32_t*)((char *)_sc->base + _reg)
#define	WRITE4(_sc, _reg, _val)	\
	*(volatile uint32_t*)((char *)_sc->base + _reg) = _val

#define SPIBUS_TRANSFER(_sc, _dev, _cmd) \
	spi_transfer(_sc, _cmd)
#define device_t	void *

/* XXX crude */
#include "mx25l.c"

#define pci_addr_range		0x04000000
#define TC_LITE_PCIE_BASE	0x02000000
#define TC_IMAGE_VER_OFFS	0x00020000
#define image_verion_addr 0x00020000

#define AXI_QSPI_FLASH_OFFSET	0x00310000

/* From FreeBSD sys/dev/xilinx/axi_quad_spi.c */
#define	SPI_SRR		0x40		/* Software reset register */
#define	 SRR_RESET	0x0A		/* The only reset value */
#define	SPI_CR		0x60		/* Control register */
#define	 CR_LSB_FIRST	(1 << 9)	/* LSB first */
#define	 CR_MASTER_TI	(1 << 8)	/* Master Transaction Inhibit */
#define	 CR_MSS		(1 << 7)	/* Manual Slave Select */
#define	 CR_RST_RX	(1 << 6)	/* RX FIFO Reset */
#define	 CR_RST_TX	(1 << 5)	/* TX FIFO Reset */
#define	 CR_CPHA	(1 << 4)	/* Clock phase */
#define	 CR_CPOL	(1 << 3)	/* Clock polarity */
#define	 CR_MASTER	(1 << 2)	/* Master (SPI master mode) */
#define	 CR_SPE		(1 << 1)	/* SPI system enable */
#define	 CR_LOOP	(1 << 0)	/* Local loopback mode */
#define	SPI_SR		0x64		/* Status register */
#define	 SR_TX_FULL	(1 << 3)	/* Transmit full */
#define	 SR_TX_EMPTY	(1 << 2)	/* Transmit empty */
#define	 SR_RX_FULL	(1 << 1)	/* Receive full */
#define	 SR_RX_EMPTY	(1 << 0)	/* Receive empty */
#define	SPI_DTR		0x68		/* Data transmit register */
#define	SPI_DRR		0x6C		/* Data receive register */
#define	SPI_SSR		0x70		/* Slave select register */
#define	SPI_TFOR	0x74		/* Transmit FIFO Occupancy Register */
#define	SPI_RFOR	0x78		/* Receive FIFO Occupancy Register */
#define	SPI_DGIER	0x1C		/* Device global interrupt enable register */
#define	SPI_IPISR	0x20		/* IP interrupt status register */
#define	SPI_IPIER	0x28		/* IP interrupt enable register */

int verbose;
struct spi_softc spisc;
struct mx25l_softc mx25sc;

void *pcie_base, *virt_addr;
unsigned mapped_size;

void usage(char *pname) {
	printf("%s [-d <device>] [-i] [-r] [-f <file>] [-o <offset>] [-p <pcie-addr>] [-v] [-V]\n", pname);
	printf("\t-o offset - is normally either 0, for Factory image, or 0x00400000 for updates.\n");
	printf("\t-d - /dev/spigenn.n\n");
	printf("\t-i - identify only\n");
	printf("\t-p pcie-addr - address of card, retrieved with pciconf -lb\n");
	printf("\t-r - read from flash and write to file\n");
	printf("\t-v - verbose\n");
	printf("\t-V - verify after write\n");
	exit(-1);
}

int main(int argc, char**argv) {
	int ch, err, fd, identify = 0, readflash = 0, verify = 0;
	char *fname = NULL, *dname = DEFAULTDEV;
	uint32_t fl_offset = 0;
	off_t target = 0;
	struct spi_softc *spsc = &spisc;
	struct mx25l_softc *mxsc = &mx25sc;
    
	while((ch = getopt(argc, argv, "d:f:ip:o:rvV")) != -1)
		switch(ch) {
		case 'd':
			dname = optarg;
			break;
		case 'f':
			fname = optarg;
			break;
		case 'i':
			identify++;
			break;
		case 'r':
			readflash++;
			break;
		case 'o':
			fl_offset = strtol(optarg, NULL, 0);
			break;
		case 'p':
			if (strlen(optarg) != 10) {
				printf("PCIe Base Address must have following format: -p 0xA0000000\n");
				usage(argv[0]);
			}
			target = (off_t)strtol(optarg, NULL, 0);
			break;
		case 'V':
			verify++;
			break;
		case 'v':
			verbose++;
			break;
		}
	if(optind < argc)
		usage(argv[0]);
	spsc->spidev = -1;
	if (target == 0) {
		int fd = open(dname, O_RDWR);
		if (fd == -1)
			usage(argv[0]);
		spsc->spidev = fd;
	}
	if (fname) {
		if (target != 0)
			printf("going to flash, pcie %X, file %s, starting at offset 0x%X\n",
			    (uint32_t)target, fname, fl_offset);
		else
			printf("going to flash, %s, file %s, starting at offset 0x%X\n",
			    dname, fname, fl_offset);
	}
	if ((target == 0) && (spsc->spidev == -1))
		usage(argv[0]);
	if ((identify == 0) && (fname == NULL)) {
		printf("-i %d, fname %s\n", identify, fname);
		usage(argv[0]);
	}

	if (target) {
		fd = pcie_map(target);

		if (pcie_base == MAP_FAILED) {
			perror("Could not mmap");
			return 1;
		}
		spsc->base = pcie_base;

		if (READ4(spsc, TC_IMAGE_VER_OFFS) == 0xffffffff) {
			spsc->base += TC_LITE_PCIE_BASE;
			if (READ4(spsc, TC_IMAGE_VER_OFFS) == 0xffffffff) {
				printf("Cannot find card\n");
				exit(1);
			}
		}
		spsc->base += AXI_QSPI_FLASH_OFFSET;

		printf("pcie_base %p\n", pcie_base);

		if (verbose)
			printaxiregs(spsc);

		err = axi_init(spsc);
		if (err) {
			printf("axi_init failed %d\n", err);
			exit(err);
		}
	}

	mxsc->sc_parent = spsc;

	err = mx25l_init(mxsc);
	if (err) {
		printf("mx25l_init failed %d\n", err);
		exit(err);
	}
	if (identify)
		exit (0);

	if (readflash) {
		err = read_flash(mxsc, fl_offset, fname);
		printf("read_flash %d\n", err);
	} else {
		err = write_flash(mxsc, fl_offset, verify, fname);
		printf("write_flash %d\n", err);
	}

	if (target) {
		munmap(pcie_base, mapped_size);
		close(fd);
	}
	return 0;
}

static int read_flash(struct mx25l_softc *sc, uint32_t fl_offset, char *fname)
{
	int bytesrd, err, flfd;
	int flsize = 16 * 1024 * 1024;
	ssize_t wrval;
	u_char buf[4096];


	if((flfd = open(fname, O_CREAT|O_WRONLY)) == -1) {
		perror("write file open failed");
		return flfd;
	}
	printf("reading");
	fflush(stdout);
	for (bytesrd = 0; bytesrd < flsize;) {
		err = mx25l_read(sc, bytesrd, (caddr_t)buf, 4096);
		if (err) {
			printf("mx25l_read err %d\n", err);
			return err;
		}
		wrval = write(flfd, buf, 4096);
		if (wrval != 4096) {
			perror("writing to file");
			return wrval;
		}

		bytesrd += wrval;

		if ((bytesrd % (4096 * 32)) == 0) {
			printf(".");
			fflush(stdout);
		}
	}
	printf("\n");
	close(flfd);
	return 0;
}

static int write_flash(struct mx25l_softc *sc, uint32_t fl_offset, int verify, char *fname)
{
	int err, flfd, i;
	uint32_t fsize;
	size_t msize;
	ssize_t rdsz;
	struct timespec tstart, tstop, tdiff;
	struct stat st;
	u_char *fimg, *rimg;

	if((flfd = open(fname, O_RDONLY)) == -1) {
		perror("flash image open failed");
		return flfd;
	}
	err = fstat(flfd, &st);
	if (err) {
		perror("stat failed");
		return err;
	}

	fsize = st.st_size;
	msize = fsize;
	printf("%s size %u, erase size %u, alloc size %zu\n", fname, fsize, sc->sc_erasesize, msize);

	fimg = calloc(1, msize);
	if (fimg == NULL) {
		perror("fimg calloc failed");
		return -1;
	}

	rimg = calloc(1, msize);
	if (rimg == NULL) {
		perror("fimg calloc failed");
		return -1;
	}
	rdsz = read(flfd, fimg, fsize);
	printf("read %zd bytes\n", rdsz);
	if (rdsz != fsize) {
		perror("read wrong length");
		return -1;
	}
	close(flfd);

	printf("mx25l_write start\n");
	clock_gettime(CLOCK_REALTIME_PRECISE, &tstart);
	err = mx25l_write(sc, fl_offset, (caddr_t)fimg, msize, 100*1024);
	if (err) {
		printf("mx25l_write err %d\n", err);
		return err;
	}
	clock_gettime(CLOCK_REALTIME_PRECISE, &tstop);
	tdiff.tv_sec = tstop.tv_sec - tstart.tv_sec;
	if (tstart.tv_nsec > tstop.tv_nsec) {
		tstop.tv_nsec += 1000000000;
		tdiff.tv_sec--;
	}
	tdiff.tv_nsec = tstop.tv_nsec - tstart.tv_nsec;
	tdiff.tv_nsec /= 100000000;
	printf("\nmx25l_write end in %lu.%01lus\n", tdiff.tv_sec, tdiff.tv_nsec);

	if (verify) {
		printf("verifying");
		fflush(stdout);
		clock_gettime(CLOCK_REALTIME_PRECISE, &tstart);
		err = mx25l_read(sc, fl_offset, (caddr_t)rimg, msize);
		if (err) {
			printf("mx25l_read err %d\n", err);
			return err;
		}
		clock_gettime(CLOCK_REALTIME_PRECISE, &tstop);
		tdiff.tv_sec = tstop.tv_sec - tstart.tv_sec;
		if (tstart.tv_nsec > tstop.tv_nsec) {
			tstop.tv_nsec += 1000000000;
			tdiff.tv_sec--;
		}
		tdiff.tv_nsec = tstop.tv_nsec - tstart.tv_nsec;
		tdiff.tv_nsec /= 100000000;
		err = 0;
		for (i = 0; i < msize; i++) {
			if (fimg[i] != rimg[i]) {
				printf("... error offset %u, expected %02X, but %02X\n", i, fimg[i], rimg[i]);
				err = 1;
				break;
			}
		}
		if (err == 0)
			printf(" %zu bytes OK in %lu.%01lus.\n", msize, tdiff.tv_sec, tdiff.tv_nsec);
	}
	free(fimg);
	free(rimg);
	return err;
}

static void printaxiregs(struct spi_softc *sc)
{
    uint32_t read_result;

    read_result = READ4(sc, SPI_CR);
    printf("SPI Control Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_SR);
    printf("SPI Status Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_SSR);
    printf("SPI Slave Select Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_TFOR);
    printf("SPI Transmit FIFO Occupancy Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_RFOR);
    printf("SPI Receive FIFO Occupancy Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_DGIER);
    printf("SPI Device global interrupt enable Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_IPISR);
    printf("SPI IP interrupt status Reg = 0x%X\n", (unsigned int)read_result);

    read_result = READ4(sc, SPI_IPIER);
    printf("SPI IP interrupt enable Reg = 0x%X\n", (unsigned int)read_result);
}


static int pcie_map(off_t target)
{
	unsigned page_size, offset_in_page;
	uint32_t width = 32;
	int fd;

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
		perror("/dev/mem open failed");
	mapped_size = page_size = getpagesize();
    
	offset_in_page = (unsigned)target & (page_size - 1);


	if (offset_in_page + width > page_size) {
		/* This access spans pages.
		 * Must map two pages to make it possible:
		 */
		mapped_size *= 2;
	}
	mapped_size = pci_addr_range;

	pcie_base = mmap(0, mapped_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, target & ~(off_t)(page_size - 1));

	return fd;
}

static int axi_init(struct spi_softc *sc)
{
	uint32_t reg;
	/* Reset */
	WRITE4(sc, SPI_SRR, SRR_RESET);

	if (READ4(sc, SPI_CR) != 0x180 || READ4(sc, SPI_SR) != 0xA5) {
		printaxiregs(sc);
		return EIO;
	}
#if 1
	/*
	 * After an FPGA load or reset, something is left such that
	 * the first flash command returns 0xff bytes. Twiddle things
	 * a bit to get rid of that.
	 */
	reg = (CR_MASTER_TI | CR_MASTER | CR_MSS | CR_SPE | CR_RST_RX | CR_RST_TX);
	WRITE4(sc, SPI_CR, reg);
	WRITE4(sc, SPI_SSR, ~1);
	WRITE4(sc, SPI_DTR, 0x66);
	reg = READ4(sc, SPI_CR);
	reg &= ~CR_MASTER_TI;
	WRITE4(sc, SPI_CR, reg);

	WRITE4(sc, SPI_SRR, SRR_RESET);
#endif

	reg = (CR_MASTER_TI | CR_MASTER | CR_MSS | CR_SPE | CR_RST_RX | CR_RST_TX);
	WRITE4(sc, SPI_CR, reg);
	WRITE4(sc, SPI_DGIER, 0);	/* Disable interrupts */

	return (0);
}

static int
spi_tx(struct spi_softc *sc, uint8_t *out_buf,
    uint8_t *in_buf, int bufsz, int cs)
{
	uint32_t i;

	for (i = 0; i < bufsz; i++) {
		if (READ4(sc, SPI_SR) & SR_TX_FULL) {
			break;
		}
		WRITE4(sc, SPI_DTR, out_buf[i]);
	}

	return (i);
}

static int
spi_rx(struct spi_softc *sc, uint8_t *out_buf,
    uint8_t *in_buf, int bufsz, int cs)
{
	uint32_t data;
	uint32_t i;

	for (i = 0; i < bufsz; i++) {
		if (READ4(sc, SPI_SR) & SR_RX_EMPTY) {
			break;
		}
		data = READ4(sc, SPI_DRR);
		if (in_buf)
			in_buf[i] = (data & 0xff);
	}

	return (i);
}

/*
 * Direct version
 */
static int
spi_transferd(struct spi_softc *sc, struct spi_command *cmd)
{
	uint32_t reg;
	uint32_t cs;
	int rxlen, txtot, txlen;


	/* testing clearing the fifos */
	reg = READ4(sc, SPI_CR);
	reg |= (CR_RST_RX | CR_RST_TX);
	WRITE4(sc, SPI_CR, reg);

	/* get the proper chip select */
	cs = 0;

	cs &= ~SPIBUS_CS_HIGH;

	/* Assert CS */
	reg = READ4(sc, SPI_SSR);
	reg &= ~(1 << cs);
	WRITE4(sc, SPI_SSR, reg);

	/* Command, transfer in chunks of fifo space */
	txtot = 0;
	while (txtot < cmd->tx_cmd_sz) {
		txlen = spi_tx(sc, cmd->tx_cmd + txtot, cmd->rx_cmd + txtot, cmd->tx_cmd_sz - txtot, cs);

		/* Start transmision */
		reg = READ4(sc, SPI_CR);
		reg &= ~CR_MASTER_TI;
		WRITE4(sc, SPI_CR, reg);

		/* Wait for TX Empty */
		while(!(READ4(sc, SPI_SR) & SR_TX_EMPTY))
			continue;
		/* Stop transmission */
		reg |= CR_MASTER_TI;
		WRITE4(sc, SPI_CR, reg);

		/* Command */
		rxlen = spi_rx(sc, cmd->tx_cmd + txtot, cmd->rx_cmd + txtot, txlen, cs);
		if (rxlen != txlen) {
			printf("Oops: TX (%d) RX (%d) mismatch\n", txlen, rxlen);
			break;
		}
		txtot += txlen;
	}

	/* Data, transfer in chunks of fifo space */
	txtot = 0;
	while (txtot < cmd->tx_data_sz) {
		txlen = spi_tx(sc, cmd->tx_data + txtot, cmd->rx_data + txtot, cmd->tx_data_sz - txtot, cs);

		/* Start transmision */
		reg = READ4(sc, SPI_CR);
		reg &= ~CR_MASTER_TI;
		WRITE4(sc, SPI_CR, reg);

		/* Wait for TX Empty */
		while(!(READ4(sc, SPI_SR) & SR_TX_EMPTY))
			continue;
		/* Stop transmission */
		reg |= CR_MASTER_TI;
		WRITE4(sc, SPI_CR, reg);

		/* Data */
		rxlen = spi_rx(sc, cmd->tx_data + txtot, cmd->rx_data + txtot, txlen, cs);
		if (rxlen != txlen) {
			printf("Oops: TX (%d) RX (%d) mismatch\n", txlen, rxlen);
			break;
		}
		txtot += txlen;
	}

	/* Deassert CS */
	reg = READ4(sc, SPI_SSR);
	reg |= (1 << cs);
	WRITE4(sc, SPI_SSR, reg);

	return (0);
}

static int
spi_transferi(struct spi_softc *sc, struct spi_command *cmd)
{
	int err;
	void *txcmd, *txdata;
	struct spigen_transfer spi;

	txcmd = NULL;
	txdata = NULL;
	bzero(&spi, sizeof(spi));       /* zero structure first */

	if ((cmd->tx_cmd_sz != cmd->rx_cmd_sz) || (cmd->tx_data_sz != cmd->rx_data_sz))
		fprintf(stderr, "xfer cmd tx %p %d, rx %p %d, data tx %p %d, rx %p %d\n",
		    cmd->tx_cmd, cmd->tx_cmd_sz, cmd->rx_cmd, cmd->rx_cmd_sz,
		    cmd->tx_data, cmd->tx_data_sz, cmd->rx_data, cmd->rx_data_sz);

	/* process cmd */
	if ((cmd->rx_cmd == NULL) && (cmd->tx_cmd == NULL)) {
		/* Leave spi.st_command.iov_base NULL */
	} else if (cmd->rx_cmd == NULL) {
		txcmd = malloc(cmd->tx_cmd_sz);
		memcpy(txcmd, cmd->tx_cmd, cmd->tx_cmd_sz);
		spi.st_command.iov_base = txcmd;
	} else {
		spi.st_command.iov_base = cmd->rx_cmd;
		memcpy(cmd->rx_cmd, cmd->tx_cmd, cmd->tx_cmd_sz);
	}
	spi.st_command.iov_len = cmd->tx_cmd_sz;
	if (cmd->rx_cmd_sz > cmd->tx_cmd_sz)
		spi.st_command.iov_len = cmd->rx_cmd_sz;

	/* process data */
	if ((cmd->rx_data == NULL) && (cmd->tx_data)) {
		/* Leave spi.st_data.iov_base NULL */
	} else if (cmd->rx_data == NULL) {
		txdata = malloc(cmd->tx_data_sz);
		memcpy(txdata, cmd->tx_data, cmd->tx_data_sz);
		spi.st_data.iov_base = txdata;
	} else {
		spi.st_data.iov_base = cmd->rx_data;
		memcpy(cmd->rx_data, cmd->tx_data, cmd->tx_data_sz);
	}
	spi.st_data.iov_len = cmd->tx_data_sz;
	if (cmd->rx_data_sz > cmd->tx_data_sz)
		spi.st_data.iov_len = cmd->rx_data_sz;

	err = ioctl(sc->spidev, SPIGENIOC_TRANSFER, &spi);

	if (txcmd)
		free(txcmd);
	if (txdata)
		free(txdata);
	if (err)
		fprintf(stderr, "Error performing SPI transaction, errno=%d\n", errno);
	return err;
}

static int
spi_transfer(struct spi_softc *sc, struct spi_command *cmd)
{
	if (sc->spidev == -1)
		return spi_transferd(sc,cmd);

	return spi_transferi(sc,cmd);
}
