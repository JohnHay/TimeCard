/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 John Hay
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
 * FreeBSD driver for the OCP TAP TimeCard
 *
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/uio.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>
#include <sys/timepps.h>
#include <sys/timetc.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <timecard.h>
#include <timecard_bus.h>
#include <timecard_reg.h>

#define TC_VERSION		0x00000003

struct timecard_corelist {
	uint32_t cl_core;
	uint32_t cl_instance;
	uint32_t cl_version;
	uint32_t cl_offs_start;
	uint32_t cl_offs_end;
	uint32_t cl_intr_num;
	uint32_t cl_sensitivity;
	char cl_magic[TC_CORE_MAGIC_SIZE];
};

struct timecard_port {
	struct timecard_bar	*p_bar;
	struct resource		*p_rres;
	device_t		p_dev;
	int			p_type;
	int			p_rclk;
	uint32_t		p_ignore;
	uint32_t		p_indx;
	uint32_t		p_offs_start;
	uint32_t		p_offs_end;
	uint32_t		p_intr_num;
	struct timecard_corelist *p_cl;
};


/* Using the order in SOM/FPGA/Readme.cl_pdf */
static struct timecard_corelist dummy_corelist[] = {
	/* Should we have different versions for the Lite PCIe and Xilinx versions? */
	{
	.cl_core = 0x00010000,
	.cl_instance = 0x0,
	.cl_version = 0x02090000,
	.cl_offs_start = 0x00010000,
	.cl_offs_end = 0x00010FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "Xilinx AXI PCIe",
	},
	{
	.cl_core = 0x00010006,
	.cl_instance = 0x0,
	.cl_version = 0x02090000,
	.cl_offs_start = 0x00010000,
	.cl_offs_end = 0x00010FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "Enjoy-Digital AXI Lite PCIe",
	},
	{
	.cl_core = 0x0000000A,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x00020000,
	.cl_offs_end = 0x00020FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC FPGA Version",
	},
	{
	.cl_core = 0x00010001,
	.cl_instance = 0x0,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00100000,
	.cl_offs_end = 0x00100FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "Xilinx AXI GPIO ext",
	},
	{
	.cl_core = 0x00010001,
	.cl_instance = 0x1,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00110000,
	.cl_offs_end = 0x00110FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "Xilinx AXI GPIO GNSS MAC",
	},
	{
	.cl_core = 0x00000007,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x00130000,
	.cl_offs_end = 0x00130FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Clock Detector",
	},
	{
	.cl_core = 0x00000008,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x00140000,
	.cl_offs_end = 0x00143FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC SMA Selector Inst0 Slave1",
	},
	/* SMA Status 0x00142000 ? */
	{
	.cl_core = 0x00010002,
	.cl_instance = 0x0,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00150000,
	.cl_offs_end = 0x0015FFFF,
	.cl_intr_num = 0x00000007,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI IIC",
	},
	{
	.cl_core = 0x00010003,
	.cl_instance = 0x0,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00160000,
	.cl_offs_end = 0x0016FFFF,
	.cl_intr_num = 0x00000003,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI UART GNSS 1",
	},
	{
	.cl_core = 0x00010003,
	.cl_instance = 0x1,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00170000,
	.cl_offs_end = 0x0017FFFF,
	.cl_intr_num = 0x00000004,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI UART GNSS 2",
	},
	{
	.cl_core = 0x00010003,
	.cl_instance = 0x2,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00180000,
	.cl_offs_end = 0x0018FFFF,
	.cl_intr_num = 0x00000005,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI UART MAC",
	},
	{
	.cl_core = 0x00010003,
	.cl_instance = 0x3,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00190000,
	.cl_offs_end = 0x0019FFFF,
	.cl_intr_num = 0x0000000A,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI UART NMEA",
	},
	/* Not in Readme list */
	{
	.cl_core = 0x00010002,
	.cl_instance = 0x1,
	.cl_version = 0x02000000,
	.cl_offs_start = 0x00200000,
	.cl_offs_end = 0x00200FFF,
	.cl_intr_num = 0x00000005,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI IIC CLOCK",
	},
	{
	.cl_core = 0x00000008,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x00220000,
	.cl_offs_end = 0x00223FFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC SMA Selector Inst0 Slave2",
	},
	{
	.cl_core = 0x00010004,
	.cl_instance = 0x0,
	.cl_version = 0x03000000,
	.cl_offs_start = 0x00300000,
	.cl_offs_end = 0x0030FFFF,
	.cl_intr_num = 0x00000008,
	.cl_sensitivity = 0x00000003,
	.cl_magic = "Xilinx AXI HWICAP",
	},
	{
	.cl_core = 0x00010005,
	.cl_instance = 0x0,
	.cl_version = 0x03020000,
	.cl_offs_start = 0x00310000,
	.cl_offs_end = 0x0031FFFF,
	.cl_intr_num = 0x00000009,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "Xilinx AXI Quad SPI flash",
	},
	{
	.cl_core = 0x00000002,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01000000,
	.cl_offs_end = 0x0100FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Adj Clock",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01010000,
	.cl_offs_end = 0x0101FFFF,
	.cl_intr_num = 0x00000001,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper GNSS PPS",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x1,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01020000,
	.cl_offs_end = 0x0102FFFF,
	.cl_intr_num = 0x00000002,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper 1",
	},
	{
	.cl_core = 0x00000005,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01030000,
	.cl_offs_end = 0x0103FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC PPS Generator",
	},
	{
	.cl_core = 0x0000000B,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01040000,
	.cl_offs_end = 0x0104FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC PPS Slave",
	},
	{
	.cl_core = 0x0000000C,
	.cl_instance = 0x0,
	.cl_version = 0x00020003,
	.cl_offs_start = 0x01050000,
	.cl_offs_end = 0x0105FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC ToD Slave",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x2,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01060000,
	.cl_offs_end = 0x0106FFFF,
	.cl_intr_num = 0x00000006,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper 2",
	},
	{
	.cl_core = 0x0000000D,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01070000,
	.cl_offs_end = 0x0107FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC IRIG Slave",
	},
	{
	.cl_core = 0x0000000D,
	.cl_instance = 0x1,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01080000,
	.cl_offs_end = 0x0108FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC IRIG Master",
	},
	{
	.cl_core = 0x0000000D,
	.cl_instance = 0x2,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01090000,
	.cl_offs_end = 0x0109FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC DCF Slave",
	},
	{
	.cl_core = 0x0000000D,
	.cl_instance = 0x3,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010A0000,
	.cl_offs_end = 0x010AFFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC DCF Master",
	},
	{
	.cl_core = 0x0000000D,
	.cl_instance = 0x4,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010B0000,
	.cl_offs_end = 0x010BFFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC TOD Master",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x3,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010C0000,
	.cl_offs_end = 0x010CFFFF,
	.cl_intr_num = 0x00000000,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper FPGA PPS",
	},
	{
	.cl_core = 0x00000003,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010D0000,
	.cl_offs_end = 0x010DFFFF,
	.cl_intr_num = 0x0000000B,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Generator 1",
	},
	{
	.cl_core = 0x00000003,
	.cl_instance = 0x1,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010E0000,
	.cl_offs_end = 0x010EFFFF,
	.cl_intr_num = 0x0000000C,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Generator 2",
	},
	{
	.cl_core = 0x00000003,
	.cl_instance = 0x2,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x010F0000,
	.cl_offs_end = 0x010FFFFF,
	.cl_intr_num = 0x0000000D,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Generator 3",
	},
	{
	.cl_core = 0x00000003,
	.cl_instance = 0x3,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01100000,
	.cl_offs_end = 0x0110FFFF,
	.cl_intr_num = 0x0000000E,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Generator 4",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x4,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01110000,
	.cl_offs_end = 0x0111FFFF,
	.cl_intr_num = 0x0000000F,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper 3",
	},
	{
	.cl_core = 0x00000004,
	.cl_instance = 0x5,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01120000,
	.cl_offs_end = 0x0112FFFF,
	.cl_intr_num = 0x00000010,
	.cl_sensitivity = 0x00000002,
	.cl_magic = "TC Sig Timestamper 4",
	},
	{
	.cl_core = 0x00000006,
	.cl_instance = 0x0,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01200000,
	.cl_offs_end = 0x0120FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Freq Counter 1",
	},
	{
	.cl_core = 0x00000006,
	.cl_instance = 0x1,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01210000,
	.cl_offs_end = 0x0121FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Freq Counter 2",
	},
	{
	.cl_core = 0x00000006,
	.cl_instance = 0x2,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01220000,
	.cl_offs_end = 0x0122FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Freq Counter 3",
	},
	{
	.cl_core = 0x00000006,
	.cl_instance = 0x3,
	.cl_version = 0x00010000,
	.cl_offs_start = 0x01230000,
	.cl_offs_end = 0x0123FFFF,
	.cl_intr_num = 0xFFFFFFFF,
	.cl_sensitivity = 0xFFFFFFFF,
	.cl_magic = "TC Freq Counter 4",
	},
};

struct timecard_bar {
	struct resource		*b_res;
	int			b_rid;
	int			b_type;
};

struct timecard_status_bits {
	bool clk_in_sync;
	bool clk_in_holdover;
	int32_t clk_offset;
	int32_t clk_drift;
	bool tod_parse_error;
	bool tod_checksum_error;
	bool tod_uart_error;
	int8_t tod_utc_offset;
	bool tod_utc_valid;
	bool tod_leap_announce;
	bool tod_leap_59;
	bool tod_leap_61;
	bool tod_leap_valid;
	int32_t tod_time_to_leap_sec;
	bool gnss_fix_ok;
	uint8_t gnss_fix;
	uint8_t gnss_sat_num_seen;
	uint8_t gnss_sat_num_locked;
	bool pps_slave_period_error;
	bool pps_slave_pulse_width_error;
};

/*
 * Version registers are 32 bit, MMmmbbbb,
 * M - 8 bits Major
 * m - 8 bits minor
 * b - 16 bits build
 * "256.256.65535\0" = 14 bytes
 *
 * Except:
 * fpga, FFFFffff
 * F - Golden image, if booted from
 * f - Normal image, if booted from
 *
 */
#define VERSION_STR_LEN		14
struct timecard_version_str {
	char driver[VERSION_STR_LEN];
	char fpga[VERSION_STR_LEN];
	char core_list[VERSION_STR_LEN];
	char adj_clock[VERSION_STR_LEN];
	char sig_gen[VERSION_STR_LEN];
	char sig_tstamper[VERSION_STR_LEN];
	char pps_gen[VERSION_STR_LEN];
	char freq_counter[VERSION_STR_LEN];
	char clock_detector[VERSION_STR_LEN];
	char sma_selector[VERSION_STR_LEN];
	char pps_source_selector[VERSION_STR_LEN];
	char pps_slave[VERSION_STR_LEN];
	char tod_slave[VERSION_STR_LEN];
	char axi_pcie[VERSION_STR_LEN];
	char axi_gpio[VERSION_STR_LEN];
	char axi_iic[VERSION_STR_LEN];
	char axi_uart[VERSION_STR_LEN];
	char axi_hwicap[VERSION_STR_LEN];
	char axi_qspi[VERSION_STR_LEN];
};

struct timecard_softc {
	device_t	sc_dev;
	struct cdev	*sc_cdev;
	struct rman	sc_iomem;

	struct timecard_bar sc_bar[1];
	int		 sc_irid;
	struct resource	*sc_ires;
	void		*sc_ihandle;
	struct mtx	sc_dev_mtx;

	int		sc_nports;
	struct timecard_corelist *sc_cl;
	struct timecard_port *sc_port;
	struct timecard_port *sc_fpga_pps_port;
	int		sc_polled;

	bool		sc_cap_msi_mask;
	int		sc_msi_vector_offset;

	int		sc_msi;
	uint32_t	sc_pci_revid;
	uint32_t	sc_base_offset;
	uint32_t	sc_clk_offset;
	uint32_t	sc_fpga_pps_offset;
	uint32_t	sc_gpio_ext_offset;
	uint32_t	sc_gpio_gnss_offset;
	uint32_t	sc_pps_slave_offset;
	uint32_t	sc_tod_offset;
	struct mtx	sc_clk_cntrl_mtx;

	struct timecard_version sc_version;
	struct timecard_version_str sc_ver_str;
	struct timecard_status sc_st;
	struct timecard_status_bits sc_st_bits;

	volatile int	sc_ts_count;
	uint32_t	sc_prev_nsec;
	uint32_t	sc_prev_sec;
	struct timespec	sc_ts_tmp;
	struct timecounter sc_tc;
	uint64_t	sc_tsc_tmp;
	u_int		sc_cpuid_tmp;

	int		sc_pps_intr_count;
	uint32_t	sc_pps_jitter;
	int		sc_pps_remove_jitter;
	int		sc_get_time_0_count;
	int		sc_get_time_X_count;
	int		sc_read_time_count;

	struct pps_state sc_pps_state;
	struct mtx	sc_pps_mtx;
};

SYSCTL_DECL(_hw_timecard);
SYSCTL_NODE(_hw, OID_AUTO, timecard, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "timecard(x) driver configuration");

static int timecard_iic_clock_enable = 1;
SYSCTL_INT(_hw_timecard, OID_AUTO, iic_clock, CTLFLAG_RDTUN,
    &timecard_iic_clock_enable, 0, "Use the IIC instead of UART to the MAC");

static int timecard_gnss1_proto = 1;
SYSCTL_INT(_hw_timecard, OID_AUTO, gnss1_proto, CTLFLAG_RDTUN,
    &timecard_gnss1_proto, 0, "GNSS1 Protocol, 0=NMEA, 1=UBX, 2=TSIP");

static int timecard_gnss1_baud = 115200;
SYSCTL_INT(_hw_timecard, OID_AUTO, gnss1_baud, CTLFLAG_RDTUN,
    &timecard_gnss1_baud, 0, "GNSS1 baud rate");

static int timecard_tc_enable = 1;
SYSCTL_INT(_hw_timecard, OID_AUTO, timecounter_enable, CTLFLAG_RDTUN,
    &timecard_tc_enable, 0, "timecounter enable");

static struct timecard_pciids {
	uint32_t device;
	const char *desc;
} timecard_ids[] = {
	{ 0x1d9b0400, "TAP TimeCard" },
};

static d_open_t		timecard_open;
static d_close_t	timecard_close;
static d_ioctl_t	timecard_ioctl;

static struct cdevsw timecard_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	timecard_open,
	.d_close =	timecard_close,
	.d_ioctl =	timecard_ioctl,
	.d_name =	"TimeCard",
};


static int timecard_detach(device_t dev);
static struct resource *
timecard_bus_alloc_resource(device_t dev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags);
static int
timecard_bus_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res);
static int
timecard_bus_get_resource(device_t dev, device_t child, int type, int rid,
    rman_res_t *startp, rman_res_t *countp);
static int
timecard_bus_setup_intr(device_t dev, device_t child, struct resource *res,
    int flags, driver_filter_t *filt, void (*ihand)(void *), void *arg, void **cookiep);
static int
timecard_bus_teardown_intr(device_t dev, device_t child, struct resource *res,
    void *cookie);
static int
timecard_bus_read_ivar(device_t dev, device_t child, int index, uintptr_t *result);
static int
timecard_bus_print_child(device_t dev, device_t child);
static int
timecard_bus_child_location(device_t dev, device_t child, struct sbuf *sb);
static int
timecard_bus_child_pnpinfo(device_t dev, device_t child, struct sbuf *sb);


const char timecard_driver_name[] = "timecard";

static MALLOC_DEFINE(M_TIMECARD, "TIMECARD", "TimeCard driver");

static int
timecard_corelist_read(struct timecard_softc *sc, int countonly)
{
	int i;
	int entries = 0;
	bus_size_t core_offs;
	struct resource *mres;
	struct timecard_bar *bar;
	struct timecard_corelist *tcl;

	bar = sc->sc_bar;
	mres = bar->b_res;

	tcl = sc->sc_cl;
	core_offs = sc->sc_base_offset + TC_CORE_LIST_BASE;

	if (bus_read_4(mres, core_offs) == 0xffffffff)
		return entries;
	for (;;entries++, core_offs += TC_CORE_ENTRY_SIZE,tcl++) {
		if (bus_read_4(mres, core_offs + 0) == TC_CORE_TYPE_END)
			break;
		if (countonly)
			continue;
		tcl->cl_core = bus_read_4(mres, core_offs + 0x00);
		tcl->cl_instance = bus_read_4(mres, core_offs + 0x04);
		tcl->cl_version = bus_read_4(mres, core_offs + 0x08);
		tcl->cl_offs_start = bus_read_4(mres, core_offs + 0x0c);
		tcl->cl_offs_end = bus_read_4(mres, core_offs + 0x10);
		tcl->cl_intr_num = bus_read_4(mres, core_offs + 0x14);
		tcl->cl_sensitivity = bus_read_4(mres, core_offs + 0x18);
		for (i = 0; i < 9; i++)
			*(uint32_t *)&(tcl->cl_magic[i*4]) = ntohl(bus_read_4(mres, core_offs + (0x1C+(i*4))));

	}
	return entries;
}

/*
 * Distingish between the different firmware versions.
 */
static int
timecard_detect(struct timecard_softc *sc)
{
	uint32_t base_offset, version;
	struct resource *mres;
	struct timecard_bar *bar;

	bar = sc->sc_bar;
	mres = bar->b_res;

	/* The Lite-PCIe based firmware offset everything TC_LITE_PCIE_BASE higher. */

	/* The pre LitePCIe PCI revid is <= 1 */
	sc->sc_pci_revid = pci_read_config(sc->sc_dev, PCIR_REVID, 1);
	if (sc->sc_pci_revid <= 1)
		base_offset = 0;
	else
		base_offset = TC_LITE_PCIE_BASE;

	version = bus_read_4(mres, base_offset + TC_IMAGE_VER_OFFS);
	if (version == 0xffffffff)
		return (EIO);

	sc->sc_base_offset = base_offset;
	sc->sc_version.fpga = version;
	device_printf(sc->sc_dev, "timecard firmware version %X\n", version);
	return 0;
}

static int
timecard_enable_intr(struct timecard_softc *sc, int intr_num)
{
	uint32_t v;

	v = bus_read_4(sc->sc_bar->b_res, TC_PCIE_MSI_ENABLE_REG);
	v |= 1 << intr_num;
	bus_write_4(sc->sc_bar->b_res, TC_PCIE_MSI_ENABLE_REG, v);
	return 0;
}

static int
timecard_disable_intr(struct timecard_softc *sc, int intr_num)
{
	uint32_t v;

	v = bus_read_4(sc->sc_bar->b_res, TC_PCIE_MSI_ENABLE_REG);
	v &= ~(1 << intr_num);
	bus_write_4(sc->sc_bar->b_res, TC_PCIE_MSI_ENABLE_REG, v);
	return 0;
}

static int
timecard_cap_tstmp(void)
{
	int mycpu;
	uint64_t tscval;

	mycpu = curcpu;
	tscval = rdtsc();
	printf("cpuid %d, tsc %lu\n", mycpu, tscval);

	return 0;
}

static int
timecard_read_time_locked(struct timecard_softc *sc, struct timespec *ts, uint64_t *tsc)
{
	int wloop = 10;
	struct resource *mres;

	mres = sc->sc_bar->b_res;

	bus_write_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG, TC_CLK_CONTROL_ENABLE | TC_CLK_CONTROL_TIME);
#ifdef NEED_READ_LOOP
	while (wloop) {
		if (bus_read_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG) & TC_CLK_CONTROL_TIME_VAL)
			break;
		wloop--;
	}
#endif
	if (tsc)
		*tsc = rdtsc();
	ts->tv_nsec = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_TIMEVALUEL_REG);
	if ((sc->sc_prev_sec == 0) || (ts->tv_nsec < sc->sc_prev_nsec))
		sc->sc_prev_sec = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_TIMEVALUEH_REG);
	ts->tv_sec = sc->sc_prev_sec;
	sc->sc_prev_nsec = ts->tv_nsec;

	sc->sc_read_time_count++;

	if (wloop == 0)
		return (EIO);
	return (0);
}

static int
timecard_read_time(struct timecard_softc *sc, struct timespec *ts, uint64_t *tsc)
{
	int error;

	mtx_lock_spin(&sc->sc_clk_cntrl_mtx);
	error = timecard_read_time_locked(sc, ts, tsc);
	mtx_unlock_spin(&sc->sc_clk_cntrl_mtx);

	return (error);
}

static u_int
timecard_get_timecount(struct timecounter *tc)
{
	u_int cntr;
	struct timecard_softc *sc = tc->tc_priv;
	struct timespec ts;
	uint64_t tsc;

	timecard_read_time(sc, &ts, &tsc);
	cntr = ts.tv_sec * 1000000000L;
	cntr += ts.tv_nsec;
	sc->sc_ts_count++;
	sc->sc_ts_tmp = ts;
	sc->sc_tsc_tmp = tsc;
	sc->sc_cpuid_tmp = curcpu;

	return (cntr);
}

static int32_t
signed_mag_2_twos_compl(uint32_t val)
{
	int32_t ret;
	if (val & (1 << 31))
		ret = 0 - (val & ~(1 << 31));
	else
		ret = val;
	return (ret);
}

static void
timecard_tod_status_clear(struct timecard_softc *sc) {
	struct resource *mres;

	mres = sc->sc_bar->b_res;
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_STATUS_REG, TC_TOD_STATUS_MASK);
}

static void
timecard_tod_uart_baud_update(struct timecard_softc *sc, uint32_t baud)
{
	uint32_t ctrl;
	struct resource *mres;

	mres = sc->sc_bar->b_res;
	ctrl = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, ctrl & ~TC_TOD_CONTROL_ENABLE);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_BAUDRATE_REG, baud);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, ctrl);
}

static void
timecard_tod_uart_polarity_update(struct timecard_softc *sc, uint32_t polarity)
{
	uint32_t ctrl;
	struct resource *mres;

	mres = sc->sc_bar->b_res;
	ctrl = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, ctrl & ~TC_TOD_CONTROL_ENABLE);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_POLARITY_REG, polarity);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, ctrl);
}

static void
timecard_get_status(struct timecard_softc *sc, struct timecard_status *st)
{
	struct resource *mres;

	mres = sc->sc_bar->b_res;

	st->clk_status = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_STATUS_REG);
	st->clk_offset = signed_mag_2_twos_compl(bus_read_4(mres, sc->sc_clk_offset + TC_CLK_STATUSOFFSET_REG));
	st->clk_drift = signed_mag_2_twos_compl(bus_read_4(mres, sc->sc_clk_offset + TC_CLK_STATUSDRIFT_REG));
	st->tod_status = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_STATUS_REG);
#if 0
	if (st->tod_status & TC_TOD_STATUS_MASK)
		bus_write_4(mres, sc->sc_tod_offset + TC_TOD_STATUS_REG, st->tod_status & TC_TOD_STATUS_MASK);
#endif
	st->tod_utc_status = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_UTC_STATUS_REG);
	st->tod_time_to_leap_sec = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_TIME_TO_LEAP_SECOND);
	st->tod_gnss_status = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_GNSS_STATUS_REG);
	st->tod_sat_num = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_SAT_NUM_REG);
	st->pps_slave_status = bus_read_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_STATUS_REG);
}

/*
 * Assume a fresh timecard_get_status()
 */
static int
timecard_update_status_bits(struct timecard_softc *sc)
{
	struct timecard_status *st;
	struct timecard_status_bits *stb;

	st = &sc->sc_st;
	stb = &sc->sc_st_bits;

	stb->clk_in_sync = (st->clk_status & TC_CLK_STATUS_IN_SYNC) == TC_CLK_STATUS_IN_SYNC;
	stb->clk_in_holdover = (st->clk_status & TC_CLK_STATUS_IN_HOLDOVER) == TC_CLK_STATUS_IN_HOLDOVER;
	stb->clk_offset = st->clk_offset;
	stb->clk_drift = st->clk_drift;
	stb->tod_parse_error = (st->tod_status & TC_TOD_STATUS_PARSE_ERR) == TC_TOD_STATUS_PARSE_ERR;
	stb->tod_checksum_error = (st->tod_status & TC_TOD_STATUS_CHECKSUM_ERR) == TC_TOD_STATUS_CHECKSUM_ERR;
	stb->tod_uart_error = (st->tod_status & TC_TOD_STATUS_UART_ERR) == TC_TOD_STATUS_UART_ERR;
	stb->tod_utc_offset = st->tod_utc_status & TC_TOD_UTC_STATUS_UTC_OFF;
	stb->tod_utc_valid = (st->tod_utc_status & TC_TOD_UTC_STATUS_UTC_VALID) == TC_TOD_UTC_STATUS_UTC_VALID;
	stb->tod_leap_announce = (st->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_ANN) == TC_TOD_UTC_STATUS_LEAP_ANN;
	stb->tod_leap_59 = (st->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_59) == TC_TOD_UTC_STATUS_LEAP_59;
	stb->tod_leap_61 = (st->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_61) == TC_TOD_UTC_STATUS_LEAP_61;
	stb->tod_leap_valid = (st->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_VALID) == TC_TOD_UTC_STATUS_LEAP_VALID;
	stb->tod_time_to_leap_sec = st->tod_time_to_leap_sec;
	if (st->tod_gnss_status & TC_TOD_GNSS_FIX_VALID) {
		stb->gnss_fix_ok = (st->tod_gnss_status & TC_TOD_GNSS_FIX_OK) == TC_TOD_GNSS_FIX_OK;
		stb->gnss_fix = (st->tod_gnss_status & TC_TOD_GNSS_FIX) >> 17;
	} else {
		stb->gnss_fix_ok = 0;
		stb->gnss_fix = 0;
	}
	if (st->tod_sat_num & TC_TOD_SAT_NUM_VALID) {
		stb->gnss_sat_num_seen = st->tod_sat_num & TC_TOD_SAT_NUM_SEEN;
		stb->gnss_sat_num_locked = (st->tod_sat_num & TC_TOD_SAT_NUM_LOCKED) >> 8;
	} else {
		stb->gnss_sat_num_seen = 0;
		stb->gnss_sat_num_locked = 0;
	}
	stb->pps_slave_period_error = st->pps_slave_status & TC_PPS_SLAVE_STATUS_PERIOD_ERR;
	stb->pps_slave_pulse_width_error = (st->pps_slave_status & TC_PPS_SLAVE_STATUS_PULSE_WIDTH_ERR) == TC_PPS_SLAVE_STATUS_PULSE_WIDTH_ERR;
	return (0);
}

/*
 * First write, then read.
 */
static void
timecard_ioctl_control(struct timecard_softc *sc, struct timecard_control *tc)
{
	struct resource *mres;
	uint32_t cntrlval = 0;

	mres = sc->sc_bar->b_res;

	if (tc->write & TC_CLK_SELECT)
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_SELECT_REG, tc->clk_select);
	if (tc->write & TC_CLK_TIME_ADJ) {
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_TIMEADJVALUEL_REG, tc->clk_time_adj_nsec);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_TIMEADJVALUEH_REG, tc->clk_time_adj_sec);
		cntrlval |= TC_CLK_CONTROL_TIME_ADJ;
	}
	if (tc->write & TC_CLK_OFFSET_ADJ) {
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_OFFSETADJVALUE_REG, tc->clk_offset_adj_value);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_OFFSETADJINTERVAL_REG, tc->clk_offset_adj_interval);
		cntrlval |= TC_CLK_CONTROL_OFFSET_ADJ;
	}
	if (tc->write & TC_CLK_DRIFT_ADJ) {
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_DRIFTADJVALUE_REG, tc->clk_drift_adj_value);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_DRIFTADJINTERVAL_REG, tc->clk_drift_adj_interval);
		cntrlval |= TC_CLK_CONTROL_DRIFT_ADJ;
	}
	if (tc->write & TC_CLK_INSYNC_THRESH)
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_INSYNCTHRESHOLD_REG, tc->clk_insync_threshold);
	if (tc->write & TC_CLK_SERVO_ADJ) {
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_SERVOOFFSETFACTORP_REG, tc->clk_servo_offset_Kp);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_SERVOOFFSETFACTORI_REG, tc->clk_servo_offset_Ki);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_SERVODRIFTFACTORP_REG, tc->clk_servo_drift_Kp);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_SERVODRIFTFACTORI_REG, tc->clk_servo_drift_Ki);
		cntrlval |= TC_CLK_CONTROL_SERVO_ADJ;
	}
	/* Add TC_CLK_CONTROL_ENABLE if any of the previous set cntrlval. */
	if (cntrlval)
		cntrlval |= TC_CLK_CONTROL_ENABLE;
	if (tc->write & TC_CLK_CONTROL)
		cntrlval |= tc->clk_control;
	if (cntrlval || (tc->write & TC_CLK_CONTROL)) {
		mtx_lock_spin(&sc->sc_clk_cntrl_mtx);
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG, cntrlval);
		mtx_unlock_spin(&sc->sc_clk_cntrl_mtx);
	}

	if (tc->write & TC_PPS_SLAVE_CONTROL)
		bus_write_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_CONTROL_REG, tc->pps_slave_control);
	if (tc->write & TC_PPS_SLAVE_STATUS_CLR)
		bus_write_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_STATUS_REG, TC_PPS_SLAVE_STATUS_PERIOD_ERR | TC_PPS_SLAVE_STATUS_PULSE_WIDTH_ERR);
	if (tc->write & TC_PPS_SLAVE_CABLE_DELAY)
		bus_write_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_CABLE_DELAY_REG, tc->pps_slave_cable_delay);
	if (tc->write & TC_TOD_CONTROL)
		bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, tc->tod_control);
	if (tc->write & TC_TOD_STATUS_CLR)
		bus_write_4(mres, sc->sc_tod_offset + TC_TOD_STATUS_REG, TC_TOD_STATUS_MASK);
	/* XXX Does TOD have to be disabled? */
	if (tc->write & TC_TOD_UART_BAUD_RATE)
		timecard_tod_uart_baud_update(sc, tc->tod_uart_baud_rate);
	if (tc->write & TC_TOD_UART_POLARITY)
		timecard_tod_uart_polarity_update(sc, tc->tod_uart_polarity);
	if (tc->write & TC_TOD_CORRECTION)
		bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CORRECTION_REG, tc->tod_correction);
	if (tc->write & TC_GPIO_EXT_GPIO2)
		bus_write_4(mres, sc->sc_gpio_ext_offset + TC_GPIO_X_GPIO2_REG, tc->gpio_ext_gpio2);
	if (tc->write & TC_GPIO_GNSS_RESET)
		bus_write_4(mres, sc->sc_gpio_gnss_offset + TC_GPIO_X_GPIO2_REG, tc->gpio_gnss_reset);

	/* Read values requested. */
	if (tc->read & TC_CLK_SELECT)
		tc->clk_select = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_SELECT_REG);
	if (tc->read & TC_CLK_TIME_ADJ) {
		tc->clk_time_adj_nsec = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_TIMEADJVALUEL_REG);
		tc->clk_time_adj_sec = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_TIMEADJVALUEH_REG);
	}
	if (tc->read & TC_CLK_OFFSET_ADJ) {
		tc->clk_offset_adj_value = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_OFFSETADJVALUE_REG);
		tc->clk_offset_adj_interval = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_OFFSETADJINTERVAL_REG);
	}
	if (tc->read & TC_CLK_DRIFT_ADJ) {
		tc->clk_drift_adj_value = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_DRIFTADJVALUE_REG);
		tc->clk_drift_adj_interval = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_DRIFTADJINTERVAL_REG);
	}
	if (tc->read & TC_CLK_INSYNC_THRESH)
		tc->clk_insync_threshold = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_INSYNCTHRESHOLD_REG);
	if (tc->read & TC_CLK_SERVO_ADJ) {
		tc->clk_servo_offset_Kp = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_SERVOOFFSETFACTORP_REG);
		tc->clk_servo_offset_Ki = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_SERVOOFFSETFACTORI_REG);
		tc->clk_servo_drift_Kp = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_SERVODRIFTFACTORP_REG);
		tc->clk_servo_drift_Ki = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_SERVODRIFTFACTORI_REG);
	}
	if (tc->read & TC_CLK_CONTROL)
		tc->clk_control = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG);

	if (tc->read & TC_PPS_SLAVE_CONTROL)
		tc->pps_slave_control = bus_read_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_CONTROL_REG);
	if (tc->read & TC_PPS_SLAVE_CABLE_DELAY)
		tc->pps_slave_cable_delay = bus_read_4(mres, sc->sc_pps_slave_offset + TC_PPS_SLAVE_CABLE_DELAY_REG);
	if (tc->read & TC_TOD_CONTROL)
		tc->tod_control = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG);
	if (tc->read & TC_TOD_UART_BAUD_RATE)
		tc->tod_uart_baud_rate = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_BAUDRATE_REG);
	if (tc->read & TC_TOD_UART_POLARITY)
		tc->tod_uart_polarity = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_POLARITY_REG);
	if (tc->read & TC_TOD_CORRECTION)
		tc->tod_correction = bus_read_4(mres, sc->sc_tod_offset + TC_TOD_CORRECTION_REG);
	if (tc->read & TC_GPIO_EXT_GPIO2)
		tc->gpio_ext_gpio2 = bus_read_4(mres, sc->sc_gpio_ext_offset + TC_GPIO_X_GPIO2_REG);
	if (tc->read & TC_GPIO_GNSS_RESET)
		tc->gpio_gnss_reset = bus_read_4(mres, sc->sc_gpio_gnss_offset + TC_GPIO_X_GPIO2_REG);
}

int
timecard_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct timecard_softc *sc;

	sc = dev->si_drv1;
	mtx_lock(&sc->sc_dev_mtx);
	device_busy(sc->sc_dev);
	mtx_unlock(&sc->sc_dev_mtx);
	return (0);
}

int
timecard_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct timecard_softc *sc;

	sc = dev->si_drv1;
	mtx_lock(&sc->sc_dev_mtx);
	device_unbusy(sc->sc_dev);
	mtx_unlock(&sc->sc_dev_mtx);
	device_printf(sc->sc_dev, "Closed.\n");
	return (0);
}

int
timecard_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
	int err = 0, loop, ts_cnt_pre, ts_cnt_post;
	struct timecard_control *tc;
	struct timecard_softc *sc;
	struct timecard_time *get_time;
	struct timecard_status *get_status;
	struct timecard_version *get_version;

	/* Look up our softc. */
	sc = dev->si_drv1;
	switch (cmd) {
	case TCIOCGETTIME:
		get_time = (struct timecard_time *)data;
		/* A side effect of nanotime is that it will leave the
		 * card time in sc_ts_tstmp, but it can be overwritten
		 * by another that also calls nanotime() or equivalent.
		 */
		loop = 10;
		while (loop > 0) {
			ts_cnt_pre = sc->sc_ts_count;
			nanotime(&get_time->kernel);
			get_time->card = sc->sc_ts_tmp;
			get_time->tsc = sc->sc_tsc_tmp;
			get_time->cpuid = sc->sc_cpuid_tmp;
			ts_cnt_post = sc->sc_ts_count;
			/* TimeCard is not the active timecounter */
			if (ts_cnt_pre == ts_cnt_post) {
				timecard_read_time(sc, &get_time->card, &get_time->tsc);
				get_time->cpuid = curcpu;
				break;
			}
			if (ts_cnt_post == (ts_cnt_pre + 1))
				break;
			loop--;
		}
		if (loop == 10)
			sc->sc_get_time_0_count++;
		else
			sc->sc_get_time_X_count++;
		break;
	case TCIOCGETSTATUS:
		get_status = (struct timecard_status *)data;
		timecard_get_status(sc, get_status);
		if (get_status->tod_status & TC_TOD_STATUS_MASK)
			timecard_tod_status_clear(sc);
		break;
	case TCIOCGETVERSION:
		get_version = (struct timecard_version *)data;
		*get_version = sc->sc_version;
		break;
	case TCIOCCONTROL:
		tc = (struct timecard_control *)data;
		timecard_ioctl_control(sc, tc);
		break;
	default:
		mtx_lock(&sc->sc_pps_mtx);
		err = pps_ioctl(cmd, data, &sc->sc_pps_state);
		mtx_unlock(&sc->sc_pps_mtx);
	}
	return (err);
}

#define TC_VER2STR(_str, _ver)	\
	snprintf(_str, sizeof(_str), "%d.%d.%d",	\
	    (_ver >> 24), (_ver >> 16) & 0xff, _ver & 0xffff)

static void
timecard_add_sysctl(struct timecard_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child, *parent;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->sc_dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->sc_dev));
	tree = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "version",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "TimeCard versions");
	parent = SYSCTL_CHILDREN(tree);

	TC_VER2STR(sc->sc_ver_str.axi_qspi, sc->sc_version.axi_qspi);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_qspi", CTLFLAG_RD, sc->sc_ver_str.axi_qspi, 0, "axi_qspi");
	TC_VER2STR(sc->sc_ver_str.axi_hwicap, sc->sc_version.axi_hwicap);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_hwicap", CTLFLAG_RD, sc->sc_ver_str.axi_hwicap, 0, "axi_hwicap");
	TC_VER2STR(sc->sc_ver_str.axi_uart, sc->sc_version.axi_uart);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_uart", CTLFLAG_RD, sc->sc_ver_str.axi_uart, 0, "axi_uart");
	TC_VER2STR(sc->sc_ver_str.axi_iic, sc->sc_version.axi_iic);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_iic", CTLFLAG_RD, sc->sc_ver_str.axi_iic, 0, "axi_iic");
	TC_VER2STR(sc->sc_ver_str.axi_gpio, sc->sc_version.axi_gpio);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_gpio", CTLFLAG_RD, sc->sc_ver_str.axi_gpio, 0, "axi_gpio");
	TC_VER2STR(sc->sc_ver_str.axi_pcie, sc->sc_version.axi_pcie);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "axi_pcie", CTLFLAG_RD, sc->sc_ver_str.axi_pcie, 0, "axi_pcie");
	TC_VER2STR(sc->sc_ver_str.tod_slave, sc->sc_version.tod_slave);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "tod_slave", CTLFLAG_RD, sc->sc_ver_str.tod_slave, 0, "tod_slave");
	TC_VER2STR(sc->sc_ver_str.pps_slave, sc->sc_version.pps_slave);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "pps_slave", CTLFLAG_RD, sc->sc_ver_str.pps_slave, 0, "pps_slave");
	TC_VER2STR(sc->sc_ver_str.pps_source_selector, sc->sc_version.pps_source_selector);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "pps_source_selector", CTLFLAG_RD, sc->sc_ver_str.pps_source_selector, 0, "pps_source_selector");
	TC_VER2STR(sc->sc_ver_str.sma_selector, sc->sc_version.sma_selector);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "sma_selector", CTLFLAG_RD, sc->sc_ver_str.sma_selector, 0, "sma_selector");
	TC_VER2STR(sc->sc_ver_str.clock_detector, sc->sc_version.clock_detector);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "clock_detector", CTLFLAG_RD, sc->sc_ver_str.clock_detector, 0, "clock_detector");
	TC_VER2STR(sc->sc_ver_str.freq_counter, sc->sc_version.freq_counter);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "freq_counter", CTLFLAG_RD, sc->sc_ver_str.freq_counter, 0, "freq_counter");
	TC_VER2STR(sc->sc_ver_str.pps_gen, sc->sc_version.pps_gen);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "pps_gen", CTLFLAG_RD, sc->sc_ver_str.pps_gen, 0, "pps_gen");
	TC_VER2STR(sc->sc_ver_str.sig_tstamper, sc->sc_version.sig_tstamper);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "sig_tstamper", CTLFLAG_RD, sc->sc_ver_str.sig_tstamper, 0, "sig_tstamper");
	TC_VER2STR(sc->sc_ver_str.sig_gen, sc->sc_version.sig_gen);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "sig_gen", CTLFLAG_RD, sc->sc_ver_str.sig_gen, 0, "sig_gen");
	TC_VER2STR(sc->sc_ver_str.adj_clock, sc->sc_version.adj_clock);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "adj_clock", CTLFLAG_RD, sc->sc_ver_str.adj_clock, 0, "adj_clock");
	TC_VER2STR(sc->sc_ver_str.core_list, sc->sc_version.core_list);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "core_list", CTLFLAG_RD, sc->sc_ver_str.core_list, 0, "core_list");
	snprintf(sc->sc_ver_str.fpga, sizeof(sc->sc_ver_str.fpga), "%d", sc->sc_version.fpga);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "fpga", CTLFLAG_RD, sc->sc_ver_str.fpga, 0, "fpga");
	snprintf(sc->sc_ver_str.driver, sizeof(sc->sc_ver_str.driver), "%d", sc->sc_version.driver);
	SYSCTL_ADD_STRING(ctx, parent, OID_AUTO, "driver", CTLFLAG_RD, sc->sc_ver_str.driver, 0, "driver");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "pps_intr_count", CTLFLAG_RD,
	    &sc->sc_pps_intr_count, 0, "Number of timecard PPS interrupts");

	SYSCTL_ADD_U32(ctx, child, OID_AUTO, "sc_pps_jitter", CTLFLAG_RD,
	    &sc->sc_pps_jitter, 0, "PPS interrupt jitter");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "sc_pps_remove_jitter", CTLFLAG_RW,
	    &sc->sc_pps_remove_jitter, 0, "Remove PPS interrupt jitter");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "get_time_0_count", CTLFLAG_RD,
	    &sc->sc_get_time_0_count, 0, "Number of timecard get_time loop 0");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "get_time_X_count", CTLFLAG_RD,
	    &sc->sc_get_time_X_count, 0, "Number of timecard get_time loop > 0");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "read_time_count", CTLFLAG_RD,
	    &sc->sc_read_time_count, 0, "Number of timecard_read_time called");

	tree = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "status",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "status");
	parent = SYSCTL_CHILDREN(tree);

	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "pps_slave_pulse_width_error", CTLFLAG_RD, &sc->sc_st_bits.pps_slave_pulse_width_error, 0, "pps_slave_pulse_width_error");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "pps_slave_period_error", CTLFLAG_RD, &sc->sc_st_bits.pps_slave_period_error, 0, "pps_slave_period_error");
	SYSCTL_ADD_U8(ctx, parent, OID_AUTO, "gnss_sat_num_locked", CTLFLAG_RD, &sc->sc_st_bits.gnss_sat_num_locked, 0, "gnss_sat_num_locked");
	SYSCTL_ADD_U8(ctx, parent, OID_AUTO, "gnss_sat_num_seen", CTLFLAG_RD, &sc->sc_st_bits.gnss_sat_num_seen, 0, "gnss_sat_num_seen");
	SYSCTL_ADD_U8(ctx, parent, OID_AUTO, "gnss_fix", CTLFLAG_RD, &sc->sc_st_bits.gnss_fix, 0, "gnss_fix");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "gnss_fix_ok", CTLFLAG_RD, &sc->sc_st_bits.gnss_fix_ok, 0, "gnss_fix_ok");
	SYSCTL_ADD_S32(ctx, parent, OID_AUTO, "tod_time_to_leap_sec", CTLFLAG_RD, &sc->sc_st_bits.tod_time_to_leap_sec, 0, "tod_time_to_leap_sec");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_leap_valid", CTLFLAG_RD, &sc->sc_st_bits.tod_leap_valid, 0, "tod_leap_valid");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_leap_61", CTLFLAG_RD, &sc->sc_st_bits.tod_leap_61, 0, "tod_leap_61");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_leap_59", CTLFLAG_RD, &sc->sc_st_bits.tod_leap_59, 0, "tod_leap_59");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_leap_announce", CTLFLAG_RD, &sc->sc_st_bits.tod_leap_announce, 0, "tod_leap_announce");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_utc_valid", CTLFLAG_RD, &sc->sc_st_bits.tod_utc_valid, 0, "tod_utc_valid");
	SYSCTL_ADD_S8(ctx, parent, OID_AUTO, "tod_utc_offset", CTLFLAG_RD, &sc->sc_st_bits.tod_utc_offset, 0, "tod_utc_offset");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_uart_error", CTLFLAG_RD, &sc->sc_st_bits.tod_uart_error, 0, "tod_uart_error");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_checksum_error", CTLFLAG_RD, &sc->sc_st_bits.tod_checksum_error, 0, "tod_checksum_error");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "tod_parse_error", CTLFLAG_RD, &sc->sc_st_bits.tod_parse_error, 0, "tod_parse_error");
	SYSCTL_ADD_S32(ctx, parent, OID_AUTO, "clk_drift", CTLFLAG_RD, &sc->sc_st_bits.clk_drift, 0, "clk_drift");
	SYSCTL_ADD_S32(ctx, parent, OID_AUTO, "clk_offset", CTLFLAG_RD, &sc->sc_st_bits.clk_offset, 0, "clk_offset");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "clk_in_holdover", CTLFLAG_RD, &sc->sc_st_bits.clk_in_holdover, 0, "clk_in_holdover");
	SYSCTL_ADD_BOOL(ctx, parent, OID_AUTO, "clk_in_sync", CTLFLAG_RD, &sc->sc_st_bits.clk_in_sync, 0, "clk_in_sync");
}

static int
timecard_init(struct timecard_softc *sc)
{
	int error, idx;
	uint32_t baud, offs_start, val;
	struct resource *mres;
	struct timecard_bar *bar;
	struct timecard_corelist *tcl;
	struct timecard_port *port;

	bar = sc->sc_bar;
	mres = bar->b_res;

	error = timecard_detect(sc);
	if (error)
		return (error);

	sc->sc_nports = timecard_corelist_read(sc, 1);
	if (sc->sc_nports == 0) {
		sc->sc_cl = dummy_corelist;
		sc->sc_nports = sizeof(dummy_corelist) / sizeof(struct timecard_corelist);
		device_printf(sc->sc_dev, "timecard_corelist_read did not find list, using hard coded list %d\n", sc->sc_nports);
	} else {
		sc->sc_cl = malloc(sc->sc_nports * sizeof(struct timecard_corelist), M_TIMECARD, M_WAITOK|M_ZERO);
		timecard_corelist_read(sc, 0);
	}
	sc->sc_port = malloc(sc->sc_nports * sizeof(struct timecard_port), M_TIMECARD, M_WAITOK|M_ZERO);
	sc->sc_version.length = sizeof(struct timecard_version);
	sc->sc_version.driver = TC_VERSION;
	/*
	 * fill in the base addresses
	 * mark the ports that should be ignored
	 * diasable interrupt sources
	 * get the versions
	 */
	for (idx = 0; idx < sc->sc_nports; idx++) {
		tcl = &sc->sc_cl[idx];
		port = &sc->sc_port[idx];
		port->p_cl = tcl;
		port->p_indx = idx;

		if (tcl->cl_offs_start > tcl->cl_offs_end) {
			device_printf(sc->sc_dev, "Broken CoreList. Please update card firmware! Wrong end offset, start 0x%X, end 0x%X, %s\n",
			    tcl->cl_offs_start, tcl->cl_offs_end, tcl->cl_magic);
			return (ENXIO);
		}
		offs_start = tcl->cl_offs_start;
		if (offs_start != 0xffffffff)
			offs_start += sc->sc_base_offset;
		port->p_offs_start = offs_start;
		port->p_offs_end = tcl->cl_offs_end;
		if (port->p_offs_end !=  0xffffffff)
			port->p_offs_end += sc->sc_base_offset;
		port->p_intr_num = tcl->cl_intr_num;

		switch (tcl->cl_core) {
		case TC_CORE_TYPE_ADJ_CLK:
			val = bus_read_4(mres, offs_start + TC_CLK_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.adj_clock = val;
				device_printf(sc->sc_dev, "CoreList version mismatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.adj_clock = tcl->cl_version;
			if (tcl->cl_instance == 0)
				sc->sc_clk_offset = offs_start;
			break;
		case TC_CORE_TYPE_TOD_SLAVE:
			val = bus_read_4(mres, offs_start + TC_TOD_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.tod_slave = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.tod_slave = tcl->cl_version;
			if (tcl->cl_instance == 0)
				sc->sc_tod_offset = offs_start;
			break;
		case TC_CORE_TYPE_AXI_GPIO:
			sc->sc_version.axi_gpio = tcl->cl_version;
			if (tcl->cl_instance == 0)
				sc->sc_gpio_ext_offset = offs_start;
			else if (tcl->cl_instance == 1)
				sc->sc_gpio_gnss_offset = offs_start;
			break;
		case TC_CORE_TYPE_SIG_GEN:
			val = bus_read_4(mres, offs_start + TC_GEN_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.sig_gen = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.sig_gen = tcl->cl_version;
			bus_write_4(mres, offs_start + 0, 0); /* Enable */
			bus_write_4(mres, offs_start + 0x34, 0); /* IRQ Mask */
			break;
		case TC_CORE_TYPE_SIG_TSTMP:
			val = bus_read_4(mres, offs_start + TC_GEN_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.sig_tstamper = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.sig_tstamper = tcl->cl_version;
			if (strncmp(tcl->cl_magic, "TC Sig Timestamper FPGA PPS", TC_CORE_MAGIC_SIZE) == 0) {
				sc->sc_fpga_pps_offset = offs_start;
				sc->sc_fpga_pps_port = port;
			}
			bus_write_4(mres, offs_start + TC_TSTMPR_CNTRL_REG, 0);
			bus_write_4(mres, offs_start + TC_TSTMPR_IRQMSK_REG, 0);
			break;
		case TC_CORE_TYPE_AXI_UART:
			sc->sc_version.axi_uart = tcl->cl_version;
			if (strncmp(tcl->cl_magic, "Xilinx AXI UART MAC", TC_CORE_MAGIC_SIZE) == 0 &&
			    timecard_iic_clock_enable)
				port->p_ignore = 1;
			bus_write_4(mres, offs_start + AXI_UART_OFFSET + 4, 0); /* IER */
			break;
		case TC_CORE_TYPE_AXI_IIC:
			sc->sc_version.axi_iic = tcl->cl_version;
			if (strncmp(tcl->cl_magic, "Xilinx AXI IIC CLOCK", TC_CORE_MAGIC_SIZE) == 0 &&
			    timecard_iic_clock_enable == 0)
				port->p_ignore = 1;
			bus_write_4(mres, offs_start + 0x1C, 0); /* GIER */
			break;
		case TC_CORE_TYPE_AXI_HWICAP:
			sc->sc_version.axi_hwicap = tcl->cl_version;
			bus_write_4(mres, offs_start + 0x1C, 0); /* GIER */
			break;
		case TC_CORE_TYPE_AXI_QSPI:
			sc->sc_version.axi_qspi = tcl->cl_version;
			bus_write_4(mres, offs_start + 0x1C, 0); /* DGIER */
			break;

		case TC_CORE_TYPE_LIST:
			sc->sc_version.axi_qspi = tcl->cl_version;
			break;
		case TC_CORE_TYPE_PPS_GEN:
			val = bus_read_4(mres, offs_start + TC_GEN_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.pps_gen = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.pps_gen = tcl->cl_version;
			break;
		case TC_CORE_TYPE_FREQ_CNT:
			val = bus_read_4(mres, offs_start + TC_GEN_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.freq_counter = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.freq_counter = tcl->cl_version;
			break;
		case TC_CORE_TYPE_CLK_DET:
			val = bus_read_4(mres, offs_start + 0x10);
			if (val > tcl->cl_version) {
				sc->sc_version.clock_detector = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.clock_detector = tcl->cl_version;
			break;
		case TC_CORE_TYPE_SMA_SEL:
			val = bus_read_4(mres, offs_start + 0x10);
			if (val > tcl->cl_version) {
				sc->sc_version.sma_selector = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.sma_selector = tcl->cl_version;
			break;
		case TC_CORE_TYPE_PPS_SEL:
			sc->sc_version.pps_source_selector = tcl->cl_version;
			break;
		case TC_CORE_TYPE_FPGA_VER:
			break;
		case TC_CORE_TYPE_PPS_SLAVE:
			val = bus_read_4(mres, offs_start + TC_GEN_VERSION_REG);
			if (val > tcl->cl_version) {
				sc->sc_version.pps_slave = val;
				device_printf(sc->sc_dev, "CoreList version missmatch 0x%X, 0x%X, %s\n",
				    tcl->cl_version, val, tcl->cl_magic);
			} else
				sc->sc_version.pps_slave = tcl->cl_version;
			sc->sc_pps_slave_offset = offs_start;
			break;
		case TC_CORE_TYPE_DUMMY:
			break;
		case TC_CORE_TYPE_AXI_PCIE:
			sc->sc_version.axi_pcie = tcl->cl_version;
			break;
		}
	}
	if (sc->sc_clk_offset == 0 || sc->sc_tod_offset == 0 ||
	    sc->sc_gpio_ext_offset == 0 || sc->sc_fpga_pps_offset == 0 ||
	    sc->sc_pps_slave_offset == 0)
		return EIO;
	/* set protocol and disable */
	val = (timecard_gnss1_proto << 28);
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, val);
	if (timecard_gnss1_baud) {
		baud = 0;
		switch (timecard_gnss1_baud) {
		case 4800: baud = 2; break;
		case 9600: baud = 3; break;
		case 19200: baud = 4; break;
		case 38400: baud = 5; break;
		case 57600: baud = 6; break;
		case 115200: baud = 7; break;
		case 230400: baud = 8; break;
		case 460800: baud = 9; break;
		}
		if (baud)
			bus_write_4(mres, sc->sc_tod_offset + TC_TOD_BAUDRATE_REG, baud);
	}
	/* clear errors */
	val = 7;
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_STATUS_REG, val);
	/* set protocol and enable */
	val = (timecard_gnss1_proto << 28) | TC_TOD_CONTROL_ENABLE;
	bus_write_4(mres, sc->sc_tod_offset + TC_TOD_CONTROL_REG, val);

	val = bus_read_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG);
	if ((val & TC_CLK_CONTROL_ENABLE) == 0) {
		val |= TC_CLK_CONTROL_ENABLE;
		bus_write_4(mres, sc->sc_clk_offset + TC_CLK_CONTROL_REG, val);
	}

	/* select IIC or UART for MAC/clock communication */
	if (timecard_iic_clock_enable)
		bus_write_4(mres, sc->sc_gpio_ext_offset + TC_GPIO_X_GPIO2_REG, 0x80000000);
	else
		bus_write_4(mres, sc->sc_gpio_ext_offset + TC_GPIO_X_GPIO2_REG, 0x00000000);

	sc->sc_pps_remove_jitter = 1;

	timecard_add_sysctl(sc);
	return 0;
}

static int
timecard_pps_ifltr(void *arg)
{
	struct timecard_softc *sc;

	sc = (struct timecard_softc *)arg;
	pps_capture(&sc->sc_pps_state);
	if (sc->sc_pps_remove_jitter && sc->sc_st_bits.clk_in_sync &&
	    timecounter == &sc->sc_tc) {
		sc->sc_pps_jitter = sc->sc_prev_nsec;
		sc->sc_pps_state.capcount = sc->sc_prev_sec * 1000000000L;
	}
	return (FILTER_SCHEDULE_THREAD);
}

static void
timecard_pps_intr(void *arg)
{
	struct timecard_softc *sc;
	struct resource *mres;

	sc = (struct timecard_softc *)arg;
	mres = sc->sc_bar->b_res;

	timecard_get_status(sc, &sc->sc_st);
	timecard_update_status_bits(sc);

	mtx_lock(&sc->sc_pps_mtx);
	pps_event(&sc->sc_pps_state, PPS_CAPTUREASSERT);
	mtx_unlock(&sc->sc_pps_mtx);

	sc->sc_pps_intr_count++;
	bus_write_4(mres, sc->sc_fpga_pps_offset + TC_TSTMPR_IRQ_REG, 1);

	return;
}

static int
timecard_probe(device_t dev)
{
	if (pci_get_vendor(dev) == 0x1d9b && pci_get_device(dev) == 0x0400) {
		device_set_desc(dev, "TimeCard");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

static int
timecard_attach(device_t dev)
{
	char buffer[64];
	int error;
	int idx, nports;
	struct timecard_bar *bar;
	struct timecard_port *port;
	struct timecard_softc *sc;
	bus_addr_t bar_start, end, ofs, start;
	bus_size_t size;
	bus_space_handle_t bsh;
	bus_space_tag_t bst;

	/* Look up our softc and initialize its fields. */
	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	sc->sc_iomem.rm_type = RMAN_ARRAY;
	error = rman_init(&sc->sc_iomem);
	if (error)
		return error;

	snprintf(buffer, sizeof(buffer), "%s I/O memory mapping",
	    device_get_nameunit(dev));
	sc->sc_iomem.rm_descr = strdup(buffer, M_TIMECARD);

	bar = sc->sc_bar;
	bar->b_rid = PCIR_BAR(0);
	bar->b_type = SYS_RES_MEMORY;
	bar->b_res = bus_alloc_resource_any(sc->sc_dev, bar->b_type, &bar->b_rid, RF_ACTIVE);
	if (bar->b_res == NULL) {
		goto fail;
	}

	start = rman_get_start(bar->b_res);
	size = rman_get_size(bar->b_res);
	end = rman_get_end(bar->b_res);
	error = rman_manage_region(&sc->sc_iomem, start, end);
	if (error)
		goto fail;

	error = timecard_init(sc);
	if (error) {
		device_printf(dev, "init failed\n");
		goto fail;
	}
	nports = sc->sc_nports;

	/* If interrupts are issued before we are ready, the kernel will panic. */ 
	pci_disable_busmaster(dev);
	sc->sc_msi_vector_offset = 1; /* irq start at 0, vector start at 1 */
	sc->sc_msi = pci_msix_count(dev);
	if (sc->sc_msi) {
		if (pci_alloc_msix(dev, &sc->sc_msi) == 0) {
			sc->sc_cap_msi_mask = true;
			sc->sc_msi_vector_offset += 32;
		} else
			sc->sc_msi = 0;
	}
	if (sc->sc_msi == 0) {
		sc->sc_msi = pci_msi_count(dev);
		if (sc->sc_msi != TC_MAX_MSI_IRQ)
			device_printf(dev, "msi_count %d\n", sc->sc_msi);
		pci_alloc_msi(dev, &sc->sc_msi);
	}
	if (sc->sc_msi == 0)
		device_printf(dev, "failed to initialize msi interrupts\n");
	error = pci_enable_busmaster(dev);
	if (error)
		device_printf(dev, "pci_enable_busmaster failed %d\n", error);

	/* PPSAPI */
	mtx_init(&sc->sc_pps_mtx, "TC PPS", NULL, MTX_DEF);
	sc->sc_pps_state.ppscap = PPS_CAPTUREASSERT;
	sc->sc_pps_state.driver_abi = PPS_ABI_VERSION;
	sc->sc_pps_state.driver_mtx = &sc->sc_pps_mtx;
	pps_init_abi(&sc->sc_pps_state);

	mtx_init(&sc->sc_clk_cntrl_mtx, "TC CLK Cntrl", NULL, MTX_SPIN);

	sc->sc_irid = sc->sc_fpga_pps_port->p_intr_num + sc->sc_msi_vector_offset;
	sc->sc_ires = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->sc_irid, RF_ACTIVE);
	if (sc->sc_ires == NULL) {
		device_printf(dev, "Failed to allocate irq\n");
		goto fail;
	}
	if (bus_setup_intr(dev, sc->sc_ires, INTR_TYPE_CLK | INTR_MPSAFE,
	    timecard_pps_ifltr, timecard_pps_intr, sc, &sc->sc_ihandle)) {
		device_printf(dev, "Can't set up interrupt\n");
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irid, sc->sc_ires);
		goto fail;
	}
	if (sc->sc_cap_msi_mask)
		timecard_enable_intr(sc, sc->sc_fpga_pps_port->p_intr_num);
	bus_write_4(bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_CNTRL_REG, 1);
	bus_write_4(bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_STS_REG, 1);
	bus_write_4(bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_IRQMSK_REG, 1);
	bus_write_4(bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_IRQ_REG, 1);
	timecard_cap_tstmp();

	for (idx = 0; idx < nports; idx++) {
		uint32_t reg_start;
		port = &sc->sc_port[idx];
		if (port->p_ignore)
			continue;

		reg_start = 0;
		switch (port->p_cl->cl_core) {
		case TC_CORE_TYPE_AXI_UART:
			port->p_type = TIMECARD_TYPE_SERIAL;
			port->p_rclk = TC_AXI_UART_CLK;
			reg_start = AXI_UART_OFFSET;
			break;
		case TC_CORE_TYPE_AXI_IIC:
			port->p_type = TIMECARD_TYPE_AXI_IIC;
			break;
		case TC_CORE_TYPE_AXI_QSPI:
			port->p_type = TIMECARD_TYPE_AXI_QSPI;
			break;
		default:
			continue;
		}

		port->p_bar = bar;
		bar_start = rman_get_start(bar->b_res);
		start = bar_start + port->p_offs_start + reg_start;
		end = bar_start + port->p_offs_end;
		ofs = start - bar_start;
		size = end - start + 1;
		device_printf(dev, "resource for %s ofs %lX sz %lX st %lX end %lX\n", port->p_cl->cl_magic, ofs, size, start, end);
		port->p_rres = rman_reserve_resource(&sc->sc_iomem,
		    start, end, size, 0, NULL);
		if (port->p_rres == NULL)
			continue;
		bsh = rman_get_bushandle(bar->b_res);
		bst = rman_get_bustag(bar->b_res);
		bus_space_subregion(bst, bsh, ofs, size, &bsh);
		rman_set_bushandle(port->p_rres, bsh);
		rman_set_bustag(port->p_rres, bst);

		port->p_dev = device_add_child(dev, NULL, -1);
		if (port->p_dev != NULL)
			device_set_ivars(port->p_dev, (void *)port);
	}

	/* Probe and attach our children. */
	for (idx = 0; idx < sc->sc_nports; idx++) {
		port = &sc->sc_port[idx];
		if (port->p_dev == NULL)
			continue;
		error = device_probe_and_attach(port->p_dev);
	}

	mtx_init(&sc->sc_dev_mtx, "TC Device", NULL, MTX_SPIN);
	sc->sc_cdev = make_dev(&timecard_cdevsw, device_get_unit(dev),
	    UID_ROOT, 123, 0660, "timecard%u", device_get_unit(dev));
	sc->sc_cdev->si_drv1 = sc;

	/* Initialize timecounter */
	sc->sc_tc.tc_get_timecount = timecard_get_timecount;
	sc->sc_tc.tc_counter_mask = ~0u;
	sc->sc_tc.tc_frequency = 1000000000;
	sc->sc_tc.tc_name = "TimeCard";
	sc->sc_tc.tc_quality = 500;
	sc->sc_tc.tc_flags = TC_FLAGS_SUSPEND_SAFE; /* True? */

	if (timecard_tc_enable) {
		sc->sc_tc.tc_priv = sc;
		tc_init(&sc->sc_tc);
		/* make it look like we are busy if timecounter is loaded. */
		mtx_lock(&sc->sc_dev_mtx);
		device_busy(sc->sc_dev);
		mtx_unlock(&sc->sc_dev_mtx);
	}

	return (0);
fail:
	timecard_detach(dev);
	return ENXIO;
}

/* Detach device. */

static int
timecard_detach(device_t dev)
{
	int idx;
	struct timecard_port *port;
	struct timecard_softc *sc;

	sc = device_get_softc(dev);

	/* Timecounters cannot be unloaded */
	if (sc->sc_tc.tc_priv != NULL)
		return ENXIO;

	/* Stop interrupts */
	pci_disable_busmaster(dev);
	/* Stop the PPS interrupts */
	bus_write_4(sc->sc_bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_CNTRL_REG, 0);
	bus_write_4(sc->sc_bar->b_res, sc->sc_fpga_pps_offset + TC_TSTMPR_IRQMSK_REG, 0);

	if (sc->sc_ires) {
		bus_teardown_intr(dev, sc->sc_ires, sc->sc_ihandle);
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irid, sc->sc_ires);
		sc->sc_ires = NULL;
	}

	/* Teardown the state in our softc created in our attach routine. */
	for (idx = 0; idx < sc->sc_nports; idx++) {
		port = &sc->sc_port[idx];
		if (port->p_dev != NULL) {
			if (device_delete_child(dev, port->p_dev) != 0)
				continue;
			port->p_dev = NULL;
		}
		if (port->p_rres != NULL)
			rman_release_resource(port->p_rres);
	}
	if (sc->sc_msi)
		pci_release_msi(dev);
	printf("free b mres\n");
	if (sc->sc_bar->b_res != NULL) {
		bus_release_resource(dev, sc->sc_bar->b_type, sc->sc_bar->b_rid, sc->sc_bar->b_res);
		sc->sc_bar->b_res = NULL;
	}
	if (sc->sc_port != NULL) {
		free(__DECONST(void *, sc->sc_port), M_TIMECARD);
		sc->sc_port = NULL;
	}
	if (sc->sc_cl != NULL) {
		if (sc->sc_cl != dummy_corelist)
			free(__DECONST(void *, sc->sc_cl), M_TIMECARD);
		sc->sc_cl = NULL;
	}
	rman_fini(&sc->sc_iomem);
	free(__DECONST(void *, sc->sc_iomem.rm_descr), M_TIMECARD);

	if (sc->sc_cdev != NULL) {
		destroy_dev(sc->sc_cdev);
		sc->sc_cdev = NULL;
	}
	if (mtx_initialized(&sc->sc_clk_cntrl_mtx))
		mtx_destroy(&sc->sc_clk_cntrl_mtx);
	if (mtx_initialized(&sc->sc_pps_mtx))
		mtx_destroy(&sc->sc_pps_mtx);
	if (mtx_initialized(&sc->sc_dev_mtx))
		mtx_destroy(&sc->sc_dev_mtx);
	printf("TimeCard detach!\n");
	return (0);
}

static struct resource *
timecard_bus_alloc_resource(device_t dev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct timecard_port *port;
	struct resource *res;
	struct timecard_softc *sc;
	device_t assigned, originator;

	/* Get our immediate child. */
	originator = child;
	while (child != NULL && device_get_parent(child) != dev)
		child = device_get_parent(child);
	if (child == NULL)
		return (NULL);

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	if (type == SYS_RES_IRQ) {
		sc = device_get_softc(dev);
		*rid = port->p_intr_num + sc->sc_msi_vector_offset;
		res = BUS_ALLOC_RESOURCE(device_get_parent(dev), dev, type, rid, start, end, count, flags | RF_SHAREABLE);
		if (res != NULL)
			start = rman_get_start(res);
		return res;
	}
	if (rid == NULL || *rid != 0)
		return (NULL);

	/* We only support default allocations. */
	if (!RMAN_IS_DEFAULT_RANGE(start, end))
		return (NULL);

	if (type == port->p_bar->b_type)
		res = port->p_rres;
	else
		return (NULL);

	if (res == NULL)
		return (NULL);

	assigned = rman_get_device(res);
	if (assigned == NULL) {	/* Not allocated */
		rman_set_device(res, originator);
		/*
		 * XXX Is this ok here? It does not work just after
		 * device_add_child() in attach()
		 */
		device_set_desc(port->p_dev, port->p_cl->cl_magic);
	} else if (assigned != originator)
		return (NULL);

	if (flags & RF_ACTIVE) {
		if (rman_activate_resource(res)) {
			if (assigned == NULL)
				rman_set_device(res, NULL);
			return (NULL);
		}
	}
	return (res);
}

static int
timecard_bus_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{
	struct timecard_port *port;
	device_t originator;

	/* Get our immediate child. */
	originator = child;
	while (child != NULL && device_get_parent(child) != dev)
		child = device_get_parent(child);
	if (child == NULL)
		return (EINVAL);

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	if (type == SYS_RES_IRQ)
		return BUS_RELEASE_RESOURCE(device_get_parent(dev), dev, type, rid, res);

	if (rid != 0 || res == NULL)
		return (EINVAL);

	if (type == port->p_bar->b_type) {
		if (res != port->p_rres)
			return (EINVAL);
	} else
		return (EINVAL);

	if (rman_get_device(res) != originator)
		return (ENXIO);
	if (rman_get_flags(res) & RF_ACTIVE)
		rman_deactivate_resource(res);
	rman_set_device(res, NULL);
	return (0);
}

static int
timecard_bus_get_resource(device_t dev, device_t child, int type, int rid,
    rman_res_t *startp, rman_res_t *countp)
{
	struct timecard_port *port;
	struct resource *res;
	rman_res_t start;

	printf("timecard_bus_get_resource type %d, rid %d, start %lu. XXX Needs rework.\n",
	    type, rid, *startp);
	if (type == SYS_RES_IRQ)
		return BUS_GET_RESOURCE(device_get_parent(dev), dev, type, rid, startp, countp);

	/* Get our immediate child. */
	while (child != NULL && device_get_parent(child) != dev)
		child = device_get_parent(child);
	if (child == NULL)
		return (EINVAL);

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	if (type == port->p_bar->b_type)
		res = port->p_rres;
	else
		return (ENXIO);

	if (rid != 0 || res == NULL)
		return (ENXIO);

	start = rman_get_start(res);
	if (startp != NULL)
		*startp = start;
	if (countp != NULL)
		*countp = rman_get_end(res) - start + 1;
	return (0);
}

static int
timecard_bus_setup_intr(device_t dev, device_t child, struct resource *res,
    int flags, driver_filter_t *filt, void (*ihand)(void *), void *arg, void **cookiep)
{
	int err;
	struct timecard_port *port;
	struct timecard_softc *sc;

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	sc = device_get_softc(dev);
	device_printf(child, "timecard_bus_setup_intr rid %d, irq %lu\n", rman_get_rid(res), rman_get_start(res));
	err = bus_setup_intr(dev, res, flags, filt, ihand, arg, cookiep);
	if (err == 0 && sc->sc_cap_msi_mask)
		timecard_enable_intr(sc, port->p_intr_num);
	if (err)
		device_printf(child, "err ret %d\n", err);
	return err;
}

static int
timecard_bus_teardown_intr(device_t dev, device_t child, struct resource *res,
    void *cookie)
{
	int err;
	struct timecard_port *port;
	struct timecard_softc *sc;

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	sc = device_get_softc(dev);
	if (sc->sc_cap_msi_mask)
		timecard_disable_intr(sc, port->p_intr_num);

	err =  bus_teardown_intr(dev, res, cookie);
	return err;
}

static int
timecard_bus_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	struct timecard_port *port;

	/* Get our immediate child. */
	while (child != NULL && device_get_parent(child) != dev)
		child = device_get_parent(child);
	if (child == NULL)
		return (EINVAL);

	port = device_get_ivars(child);
	KASSERT(port != NULL, ("%s %d", __func__, __LINE__));

	if (result == NULL)
		return (EINVAL);

	switch(index) {
	case TIMECARD_IVAR_CLOCK:
		*result = port->p_rclk;
		break;
	case TIMECARD_IVAR_TYPE:
		*result = port->p_type;
		break;
	default:
		return (ENOENT);
	}
	return (0);
}

static int
timecard_bus_print_child(device_t dev, device_t child)
{
	struct timecard_port *port;
	char *ptr;
	int retval;

	port = device_get_ivars(child);
	retval = 0;

	ptr = strstr(port->p_cl->cl_magic, "UART");
	if (ptr == NULL)
		ptr = strstr(port->p_cl->cl_magic, "IIC");
	if (ptr == NULL)
		ptr = strstr(port->p_cl->cl_magic, "SPI");
	if (ptr == NULL)
		ptr = port->p_cl->cl_magic;
	retval += bus_print_child_header(dev, child);
	retval += printf(" at port %d (%s)", port->p_indx, ptr);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

static int
timecard_bus_child_location(device_t dev, device_t child, struct sbuf *sb)
{
	struct timecard_port *port;

	port = device_get_ivars(child);
	sbuf_printf(sb, "port=%d", port->p_indx);
	return (0);
}

static int
timecard_bus_child_pnpinfo(device_t dev, device_t child, struct sbuf *sb)
{
	struct timecard_port *port;

	port = device_get_ivars(child);
	sbuf_printf(sb, "type=%d", port->p_type);
	return (0);
}

static device_method_t timecard_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			timecard_probe),
	DEVMETHOD(device_attach,		timecard_attach),
	DEVMETHOD(device_detach,		timecard_detach),

	DEVMETHOD(bus_alloc_resource,		timecard_bus_alloc_resource),
	DEVMETHOD(bus_release_resource,		timecard_bus_release_resource),
	DEVMETHOD(bus_get_resource,		timecard_bus_get_resource),
	DEVMETHOD(bus_read_ivar,		timecard_bus_read_ivar),
	DEVMETHOD(bus_setup_intr,		timecard_bus_setup_intr),
	DEVMETHOD(bus_teardown_intr,		timecard_bus_teardown_intr),
	DEVMETHOD(bus_print_child,		timecard_bus_print_child),
	DEVMETHOD(bus_child_pnpinfo,		timecard_bus_child_pnpinfo),
	DEVMETHOD(bus_child_location,		timecard_bus_child_location),

	DEVMETHOD_END
};

static driver_t timecard_driver = {
	timecard_driver_name,
	timecard_methods,
	sizeof(struct timecard_softc),
};

DRIVER_MODULE(timecard, pci, timecard_driver, 0, 0);
MODULE_PNP_INFO("W32:vendor/device;D:#", pci, timecard,
    timecard_ids, nitems(timecard_ids));
