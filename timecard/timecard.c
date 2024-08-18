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
 * Main aim:
 *    Read time directly from the TimeCard and make it available through
 *    shm for ntpd's use.
 *
 * Adjust the kernel time using ntp_adjtime().
 * Adjust the SiTime oscillator to minimise the frequency error.
 * Calculate the oscillator aging and compensate for that.
 * Regularly update a log file with statistics.
 * Regularly update a drift file with the latest oscillator offset and
 * aging values.
 *
 * STARTUP:
 * On startup, once the aging feature of the oscillator is being used,
 * cold and warm starts should be handled differently because with a warm
 * start, the oscillator will already be applying aging.
 * Read driftfile, if it exists.
 * Read XO values - xo_init()
 * Let train_init() look through and decide.
 */

#include <sys/types.h>
#include <assert.h>
#include <devinfo.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/cpuset.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/timex.h>
#include <dev/iicbus/iic.h>
#include <timecard.h>
#include <timecard_reg.h>

#ifdef USE_BME
#include "bme280.h"
#endif

#define STEP_THRESH	(2*1000*1000*1000)	/* Step if more than 2 seconds */
#define SECONDSPERDAY	(24 * 60 * 60)

/* If a timecounter uses a frequency of 1GHz the math introduces a small error. */
#define FBSD_FREQ_ERR	(1.52588e-05)
#define SCALE_FREQ	65536			/* frequency scale */
#define FREQTOPPM(x)	((double)x / SCALE_FREQ)

#define TRAIN_PERIOD		180		/* 3 minutes for now */
#define TRAIN_TOTAL_FRAC	2		/* shift fraction of offset_acc_total to add */
#define TRAIN_STABLE		(TRAIN_PERIOD / 10)	/* less than a 10th of TRAIN_PERIOD in ns */

#define MAX_TRAIN_SHIFT		1
#define MIN_TRAIN_SHIFT		0
#define MAX_TRAIN_STABLE	2

#define TEMP_COMP_CNT	30

#define MIN_KERN_SHIFT		0
#define MAX_KERN_SHIFT		8
#define MAX_KERN_STABLE		8

#define DEVLEN			20		/* device name should not be longer */
#define DEVPREF			"/dev/"
#define DEFAULTDEV		"timecard0"
#define DEFAULTIICDEV		"iic0"
#define DEFAULTIICCLKDEV	"iic1"
#define DEV_IIC_AXI		"Xilinx AXI IIC"
#define DEV_IIC_AXI_CLK		"Xilinx AXI IIC CLOCK"

/* Used for sanity checks */
#define XO_PULL_MAX		10.0E-6		/* default (10ppm) */
#define XO_AGING_MAX		10.0E-14	/* ns/s */
#define XO_POWER_MIN		0.3
#define XO_POWER_MAX		1.2

/* Register definitions of the SiT5721 DCOCXO */
#define SIT_ADDR		0x60
#define XO_PART_NUMBER		0x50	/* 256 byte ascii */
#define XO_NOM_FREQ		0x52	/* 32 byte ascii, nominal frequency */
#define XO_SERIAL_NUMBER	0x56	/* 32 byte ascii, lot and serial number */
#define XO_FAB_DATE		0x57	/* 32 byte ascii */
#define XO_PULL_VALUE		0x61    /* 32 bit float fractional offset */
#define XO_PULL_RANGE		0x62    /* 32 bit float fractional offset */
#define XO_AGE_COMP		0x63    /* 32 bit float fraction / second */
#define XO_RAMP_RATE		0x64    /* 32 bit float fraction / second (abs) */
#define XO_UPTIME		0xa0	/* 32 bit uint seconds */
#define XO_TEMP			0xa1    /* 32 bit float resonator temperature, C */
#define XO_VOLTAGE		0xa3    /* 32 bit float supply voltage, V */
#define XO_OFFSET		0xab    /* 32 bit float total offset written */
#define XO_HEATER_POWER		0xa7    /* 32 bit float heater power, Watt */
#define XO_STATUS		0xae    /* 32 bit uint error status flag, 7 is ok */
#define XO_STABILITY		0xaf	/* 32 bit uint, 1 when stable */
#define XO_TARGET_POWER		0xb1    /* 32 bit float power target, Watt */
#define XO_STATUS_CLEAR		0xe1	/* write 0x64, 0x01 to clear status */

/* From ntpd/refclock_shm.c */

#define PRECISION       (-1)    /* precision assumed (0.5 s) */
#define NSAMPLES        3       /* stages of median filter */
#define SHM_MODE_PRIVATE 0x0001

#define LEAP_NOWARNING	0x0	/* normal, no leap second warning */
#define	LEAP_ADDSECOND	0x1	/* last minute of day has 61 seconds */
#define	LEAP_DELSECOND	0x2	/* last minute of day has 59 seconds */
#define	LEAP_NOTINSYNC	0x3	/* overload, clock is free running */

struct shmTime {
	int    mode; /* 0 - if valid is set:
		      *       use values,
		      *       clear valid
		      * 1 - if valid is set:
		      *       if count before and after read of values is equal,
		      *         use values
		      *       clear valid
		      */
	volatile int    count;
	time_t		clockTimeStampSec;
	int		clockTimeStampUSec;
	time_t		receiveTimeStampSec;
	int		receiveTimeStampUSec;
	int		leap;
	int		precision;
	int		nsamples;
	volatile int    valid;
	unsigned	clockTimeStampNSec;	/* Unsigned ns timestamps */
	unsigned	receiveTimeStampNSec;	/* Unsigned ns timestamps */
	int		dummy[8];
};

struct find_arg
{
	char *devname;
	struct devinfo_dev *iic;
	struct devinfo_dev *axi_iic;
	struct devinfo_dev *timecard;
};

struct axi_iic_info {
	char *devname;
	int fd;
	uint32_t iicerrcnt;
	uint32_t iicffcnt;
	uint32_t iicxfererrcnt;
};

#ifdef USE_BME
/* Needed for bm280 driver */
struct identifier
{
	/* Variable to hold device address */
	uint8_t dev_addr;
	struct axi_iic_info *iic;
};

struct identifier bmeid;
static struct bme280_dev bmedev;
#endif

/* Store the current value every 24 hours */
#define PULLBUFSIZ	366
static float pullbuf[PULLBUFSIZ];
static time_t pulltsbuf[PULLBUFSIZ];

typedef enum {
	TC_START = 0,
	TC_INIT,
	TC_STEP,
	TC_LOSTSYNC,
	TC_LOSTFIX,
	TC_PRESYNC,
	TC_SYNC
} timeCardState;

struct timeCardStatus {
	timeCardState state;
	bool time_valid;
	bool gnss_valid;
	bool kern_stepped;
	int count;
};

struct timeCardInfo {
	struct shmTime volatile *shm;
	struct axi_iic_info iic;
	struct axi_iic_info iic_clk;

	int enable_bindcpu;
	int bindcpu;
#ifdef USE_BME
	int enable_bme;
#endif
	int enable_clock;
	int enable_clock_temp_comp;
	int enable_kernel;
	int enable_shm;
	int enable_training;

	struct timecard_version version;	/* As reported by kld */
	struct timeCardStatus status;

	char *tcdevname;
	int tcfd;

	struct timespec tcardClk;	/* time from time card */
	struct timespec rcvTstmp;	/* OS time at tcardClk */
	uint64_t tsc;
	uint64_t difftsc;
	u_int cpuid;

	int32_t clk_status_offset;
	int32_t clk_status_drift;
	uint32_t clk_clkstatus;
	uint32_t clk_pps_select;
	uint32_t pps_slave_control;
	uint32_t pps_slave_status;
	uint32_t tod_status;
	uint32_t tod_gnss_status;
	uint32_t tod_utc_status;

	time_t kernel_upd_tstmp;

	int64_t kernel_offset;
	int64_t kernel_offset_acc;
	int ntpa_status;
	long ntpa_offset;
	long ntpa_freq;
	long kern_fudge_freq;
	int tai_offset;
	int precision;
	int kern_shift;
	int kern_shift_stbl;

	int32_t drift_acc_total;
	int32_t drift_acc_total_count;
	int32_t drift_acc;
	int32_t drift_acc_count;
	int32_t offset_acc_total;
	int32_t offset_acc_total_count;
	int32_t offset_acc;
	int32_t offset_acc_count;

	int train_use_total_offset;
	int32_t train_period;
	int32_t train_stable_val;
	int32_t train_shift_max;
	int32_t train_shift_min;
	int32_t train_shift;
	int32_t train_stable_max;
	int32_t train_stable;
	float train_adj;
	float train_pull;
	time_t nexttrain;

	FILE *logf;			/* log file descriptor */
	char *logfname;
	char *driftfname;
	time_t nextdriftf;

	/* Oscilator status */
	float xo_pull;
	float xo_offset;
	float xo_aging;
	float xo_power;
	float xo_power_acc;
	float xo_power_min;
	float xo_power_max;
	float xo_power_ref;
	float temp_comp_fact;
	float temp_comp;
	int xo_power_cnt;

	float aging;			/* Aging ns/s */
	float dfaging;			/* aging read from / to driftfile */
	float dfoffset;			/* offset read from / to driftfile */

	int32_t pullbindx;	/* last entry / next to be overwritten */
	int32_t pullbcnt;	/* number of entries */
	time_t nextaging;

	uint8_t serialno[6];
	char serialnotxt[18];		/* Format 12:34:56:78:90:AB + null */
#ifdef USE_BME
	double bme_temperature;
	double bme_pressure;
	double bme_humidity;
#endif
	struct sigaction hup_action;
	struct sigaction term_action;
};

static char *pname;
static struct timeCardInfo tcInfo;
static volatile sig_atomic_t need_freopen;
static volatile sig_atomic_t need_to_die;

static void huphandler(int _val);
static void termhandler(int _val);
void bind_cpu(unsigned int cpunr);

static int findiicdevs(struct timeCardInfo *tci);
static int parse_temp_comp_arg(struct timeCardInfo *tci, char *argstr);
static int parse_train_arg(struct timeCardInfo *tci, char *argstr);
static int initdevs(struct timeCardInfo *tci);
static int readdriftfile(struct timeCardInfo *tci);
static int captureTime(struct timeCardInfo *tcInfo);
static int updateTimeStatus(struct timeCardInfo *tcInfo);
static struct shmTime* getShmTime(int unit, int/*BOOL*/ forall);
static int clearKernTime(struct timeCardInfo *tc);
static int initKernTime(struct timeCardInfo *tc);
static int updKernTime(struct timeCardInfo *tc);
static int updShmTime(struct timeCardInfo *tc);
int64_t timespecoffset(struct timespec *ts1, struct timespec *ts2);

static int axi_iic_xfer(struct axi_iic_info *iic, struct iic_msg *msgs, uint32_t num);
static int probe_eeprom(struct timeCardInfo *tci);
#ifdef USE_BME
static int probe_bme280(struct timeCardInfo *tci);
static int read_bme280(struct timeCardInfo *tci);
#endif
static int logstats(struct timeCardInfo *tcInfo);
static void timespec_sub(struct timespec *_v1, struct timespec *_v2,
    struct timespec *_result);
static int temp_comp_init(struct timeCardInfo *tci);
static int temp_comp(struct timeCardInfo *tci);
static int train_init(struct timeCardInfo *tci, int hotstart);
static int train_reset(struct timeCardInfo *tci);
static int train(struct timeCardInfo *tci);
static int xo_init(struct timeCardInfo *tci);
static int xo_stats(struct timeCardInfo *tcInfo);
static int xo_update(struct timeCardInfo *tci);
static void calcaging(struct timeCardInfo *tci);
static void driftfupdate(struct timeCardInfo *tci);

void usage(void) {
	printf("usage: %s [-B cpuid] %s[-C refv,mult] [-c] [-d driftfile] [-f /dev/timecardN] [-l logfile] [-k] [-s] [-T period,min_shift,max_shift,stable] [-t] [-v]\n",
	    pname,
#ifdef USE_BME
	    "[-b] "
#else
	    ""
#endif
	    );
	printf("\t-B cpuid: bind process to cpu\n");
#ifdef USE_BME
	printf("\t-b: enable bme environmental monitoring\n");
#endif
	printf("\t-C refv,mult: enable clock temperature compensation. Refv and mult are floats.\n");
	printf("\t-c: enable clock monitoring\n");
	printf("\t-d driftfile: file to periodically write the drift values, used to spead startup\n");
	printf("\t-f timecard device: default /dev/timecard0\n");
	printf("\t-h: hot start, assuming the drift is valid \n");
	printf("\t-k: enable kernel synchronisation\n");
	printf("\t-l logfile: file to write logs, HUP will close and reopen it\n");
	printf("\t-s: enable ntp shm shared memory interface\n");
	printf("\t-T period,shift,stable: set the period (s), min, max shift (bits) and stable (ns) for training\n");
	printf("\t-t: enable clock training\n");
	printf("\t-v: verbose\n\n");
	printf("\tdriftfile is optional and used to speed up subsequent clock convergence\n");
	printf("\tlogfile is optional and used to write per second logging. A HUP will cause the file to be reopened\n");
	exit(-1);
}

int main(int argc, char **argv)
{
	int ch, err, hotstart = 0, verbose = 0;
	struct timespec now, nextup, tsleep;
	struct shmTime volatile* shm;

	pname = argv[0];

#ifdef USE_BME
	tcInfo.enable_bme = 1;
#endif

	while((ch = getopt(argc, argv, "B:bC:cd:f:hkl:sT:tv")) != -1)
		switch(ch) {
		case 'B':
			tcInfo.enable_bindcpu = 1;
			tcInfo.bindcpu = strtol(optarg, NULL, 0);
			break;
		case 'b':
			/* Silently do nothing if not defined. */
#ifdef USE_BME
			tcInfo.enable_bme = 0;
#endif
			break;
		case 'C':
			tcInfo.enable_clock_temp_comp = parse_temp_comp_arg(&tcInfo, optarg);
			break;
		case 'c':
			tcInfo.enable_clock = 1;
			break;
		case 'd':
			tcInfo.driftfname = optarg;
			break;
		case 'f':
			tcInfo.tcdevname = optarg;
			break;
		case 'h':
			hotstart = 1;
			break;
		case 'k':
			tcInfo.enable_kernel = 1;
			break;
		case 'l':
			tcInfo.logfname = optarg;
			tcInfo.logf = fopen(optarg, "a");
			break;
		case 's':
			tcInfo.enable_shm = 1;
			break;
		case 'T':
			parse_train_arg(&tcInfo, optarg);
			break;
		case 't':
			tcInfo.enable_training = 1;
			break;
		case 'v':
			verbose++;
			break;
		default:
			printf("default hit\n");
			usage();
		}
	if(optind < argc)
		usage();

	if (tcInfo.tcdevname == NULL) {
		tcInfo.tcdevname = malloc(DEVLEN);
		sprintf(tcInfo.tcdevname, "%s%s", DEVPREF, DEFAULTDEV);
	}
	err = findiicdevs(&tcInfo);
	if (err) {
		printf("Could not find IIC devices.\n");
		exit(ENXIO);
	}
	err = initdevs(&tcInfo);
	if (err) {
		printf("Could not open all devices.\n");
		exit(ENXIO);
	}

	tcInfo.hup_action.sa_handler = huphandler;
	sigemptyset (&tcInfo.hup_action.sa_mask);
	tcInfo.hup_action.sa_flags = 0;
	sigaction (SIGHUP, &tcInfo.hup_action, NULL);

	tcInfo.term_action.sa_handler = termhandler;
	sigemptyset (&tcInfo.term_action.sa_mask);
	tcInfo.term_action.sa_flags = 0;
	sigaction (SIGINT, &tcInfo.term_action, NULL);
	sigaction (SIGTERM, &tcInfo.term_action, NULL);

	/* bind to cpu, so the same TSC is used. */
	if (tcInfo.enable_bindcpu)
		bind_cpu(tcInfo.bindcpu);

	if (tcInfo.enable_shm) {
		shm = getShmTime(0, 1);
		shm->precision = PRECISION;
		shm->valid = 0;
		shm->nsamples = NSAMPLES;
		tcInfo.shm = shm;
	}

	if (tcInfo.iic.fd != -1) {
		err = probe_eeprom(&tcInfo);
		if (err)
			printf("Error initializing EEPROM %d\n", err);
		if (verbose)
			printf("Serial Number: %s\n", tcInfo.serialnotxt);
	}
#ifdef USE_BME
	if (tcInfo.enable_bme) {
		err = probe_bme280(&tcInfo);
		if (err) {
			printf("Error initializing BME280 sensor %d\n", err);
		}
	}
#endif

	/*
	 * Determine the precision
	 * What about leap seconds?
	 */

	clock_gettime(CLOCK_REALTIME, &now);
	nextup = now;

	if (tcInfo.enable_clock) {
		if (tcInfo.driftfname != NULL)
			readdriftfile(&tcInfo);
		xo_init(&tcInfo);
		xo_stats(&tcInfo);
		if (tcInfo.enable_clock_temp_comp)
			temp_comp_init(&tcInfo);
		tcInfo.nexttrain = now.tv_sec + tcInfo.train_period;
		if (tcInfo.enable_training || tcInfo.enable_clock_temp_comp)
			train_init(&tcInfo, hotstart);
	}

	/* update drift file every 3 hours */
	tcInfo.nextdriftf = now.tv_sec + (3 * 60 * 60);
	tcInfo.nextaging = now.tv_sec + (3 * 60 * 60);

#ifdef USE_BME
	if (tcInfo.enable_bme)
		read_bme280(&tcInfo);
#endif
	if (tcInfo.enable_kernel)
		initKernTime(&tcInfo);

	if (verbose)
		printf("Starting loop %lu.%09lu\n", now.tv_sec, now.tv_nsec);

	if (verbose && tcInfo.enable_clock_temp_comp)
		printf("clock temperature compensation enabled, using reference %fW and multiplier %f\n",
			tcInfo.xo_power_ref, tcInfo.temp_comp_fact);
	fflush(stdout);

	while(1) {
		err = captureTime(&tcInfo);
		if (err == 1) {
			/* Time was stepped */
			clock_gettime(CLOCK_REALTIME, &nextup);
		}

		if (tcInfo.status.time_valid) {
			if (tcInfo.enable_shm)
				updShmTime(&tcInfo);
			if (tcInfo.enable_kernel)
				updKernTime(&tcInfo);
		}
		if (tcInfo.enable_clock)
			xo_stats(&tcInfo);
#ifdef USE_BME
		if (tcInfo.enable_bme)
			read_bme280(&tcInfo);
#endif
		logstats(&tcInfo);

		if (tcInfo.enable_clock && tcInfo.enable_clock_temp_comp)
			temp_comp(&tcInfo);
		if (tcInfo.enable_clock && tcInfo.enable_training && tcInfo.status.time_valid) {
			train(&tcInfo);
			calcaging(&tcInfo);
		}
		if (tcInfo.enable_clock && (tcInfo.enable_training ||
		  tcInfo.enable_clock_temp_comp))
			xo_update(&tcInfo);

		if (tcInfo.driftfname != NULL)
			driftfupdate(&tcInfo);

		if (need_to_die)
			break;
		clock_gettime(CLOCK_REALTIME, &now);
		nextup.tv_sec++;
		timespec_sub(&nextup, &now, &tsleep);
		nanosleep(&tsleep, NULL);
	}

	if (tcInfo.enable_kernel)
		clearKernTime(&tcInfo);

	if (tcInfo.enable_shm)
		shmdt((const void *)shm);
	return 0;

}

static void huphandler(int _val)
{
	need_freopen = 1;
}

static void termhandler(int _val)
{
	need_to_die = 1;
}

void bind_cpu(unsigned int cpunr)
{
	cpuset_t mask;

	CPU_ZERO(&mask);
	CPU_SET(cpunr, &mask);
	cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_PID, -1, sizeof(mask), &mask);
}

static int find_axi(struct devinfo_dev *dev, void *xfa)
{
	struct find_arg	*fa = xfa;
	int rv;

	if (strcmp(dev->dd_name, fa->devname) == 0) {
		fa->iic = dev;
		return (1);
	}
	rv = devinfo_foreach_device_child(dev, find_axi, xfa);
	if (rv == 1) {
		if (strstr(dev->dd_name, "axi_iic") == dev->dd_name)
			fa->axi_iic = dev;
		if (strstr(dev->dd_name, "timecard") == dev->dd_name)
			fa->timecard = dev;
	}
	return (rv);
}

static int findiicdevs(struct timeCardInfo *tci)
{
	struct find_arg		iic;
        struct devinfo_dev      *root;
	char iicdev[6]; /* "iic99\0" */
	char *tcname;
	int foundiic = 0, foundiicclk = 0, i, rv;

	tcname = basename(tci->tcdevname);

	tci->iic.devname = malloc(DEVLEN);
	tci->iic_clk.devname = malloc(DEVLEN);
#if 0
	/* Fill in some defaults */
	sprintf(tci->iic.devname, "%s%s", DEVPREF, DEFAULTIICDEV);
	sprintf(tci->iic_clk.devname, "%s%s", DEVPREF, DEFAULTIICCLKDEV);
#endif
	if ((rv = devinfo_init()) != 0) {
		errno = rv;
		fprintf(stderr, "devinfo_init failed\n");
		return rv;
	}
	if ((root = devinfo_handle_to_device(DEVINFO_ROOT_DEVICE)) == NULL) {
		fprintf(stderr, "devinfo_handle_to_device() can't find root device\n");
		return 1;
	}

	for (i = 0; i < 10; i++) {
		sprintf(iicdev, "iic%d", i);

		iic.devname = iicdev;
		iic.iic = NULL;
		iic.axi_iic = NULL;
		iic.timecard = NULL;
		if (devinfo_foreach_device_child(root, find_axi, (void *)&iic) == 0)
			break;

		if ((strncmp(iic.axi_iic->dd_desc, DEV_IIC_AXI, DEVLEN) == 0) &&
		    (strncmp(iic.timecard->dd_name, tcname, DEVLEN) == 0)) {
			sprintf(tci->iic.devname, "%s%s", DEVPREF, iic.iic->dd_name);
			foundiic = 1;
		}
		if ((strncmp(iic.axi_iic->dd_desc, DEV_IIC_AXI_CLK, DEVLEN) == 0) &&
		    (strncmp(iic.timecard->dd_name, tcname, DEVLEN) == 0)) {
			sprintf(tci->iic_clk.devname, "%s%s", DEVPREF, iic.iic->dd_name);
			foundiicclk = 1;
		}
		if (foundiic && foundiicclk)
			break;
	}
	if (!foundiic)
		printf("Did not find iic device\n");
	if (!foundiicclk)
		printf("Did not find iic_clk device\n");

	devinfo_free();
	if (foundiic && foundiicclk)
		return 0;
	return 1;
}

/*
 * expect to floats seperated with a comma ',' or space ' ', eg. "0.92,2.4".
 */
int parse_temp_comp_arg(struct timeCardInfo *tci, char *argstr)
{
        char *nxtp, *endp;
	int retv = 0;
	float a = 0, b = 0;

	tci->xo_power_min = XO_POWER_MIN;
	tci->xo_power_max = XO_POWER_MAX;

	a = strtof(argstr, &nxtp);
	if (nxtp != argstr && (*nxtp == ',' || *nxtp == ' ')) {
		nxtp++;
		b = strtof(nxtp, &endp);
		if (nxtp != endp) {
			tci->xo_power_ref = a;
			tci->temp_comp_fact = b;
			retv = 1;
		}
	}
	return retv;
}

/*
 * Expect a string period,shift,stable
 * Where all are positive integers.
 * period is the period in seconds at which traing happens. Default TRAIN_PERIOD.
 * shift is the number of steps if the frequency stays stable. Default MAX_TRAIN_SHIFT.
 * stable is the limit of stability in ns. Default TRAIN_STABLE.
 */
static int parse_train_arg(struct timeCardInfo *tci, char *argstr)
{
	int i, retv = 0;
        char *nxtp, *endp;
	uint32_t period = 0, minshift = 0, maxshift = 0, stable = 0, tval = 0;

	nxtp = argstr;
	for (i = 0; i < 4; i++) {
		tval = strtoul(nxtp, &endp, 0);
		if (nxtp == endp)
			return 0;
		switch (i) {
		case 0:
			period = tval;
			break;
		case 1:
			minshift = tval;
			break;
		case 2:
			maxshift = tval;
			break;
		case 3:
			stable = tval;
			break;
		}
		if (*endp == '\0')
			break;
		if (*endp != ',' && *endp != ' ')
			return 0;
		nxtp = endp + 1;
	}
	tci->train_period = period;
	tci->train_shift_min = minshift;
	tci->train_shift_max = maxshift;
	tci->train_stable_val = stable;
	printf("Setting train period %u, shift_min %u, shift_max %u, stable_val %u\n",
	    period, minshift, maxshift, stable);
	return retv;
}

static int initdevs(struct timeCardInfo *tci)
{
	int err = 0;
	tci->tcfd = open(tci->tcdevname, O_RDWR);
	if (tci->tcfd == -1)
		perror(tci->tcdevname);
	else
		printf("using %s", tci->tcdevname);

	tci->iic.fd = open(tci->iic.devname, O_RDWR);
	if (tci->iic.fd == -1)
		perror(tci->iic.devname);
	else
		printf(", %s", tci->iic.devname);

	tci->iic_clk.fd = open(tci->iic_clk.devname, O_RDWR);
	if (tci->iic_clk.fd == -1)
		perror(tci->iic_clk.devname);
	else
		printf(", %s (CLK)", tci->iic_clk.devname);
	printf("\n");

	if (tci->tcfd != -1) {
		err = ioctl(tci->tcfd, TCIOCGETVERSION, (caddr_t)&tci->version);
		if (err == 0)
			printf("timecard.ko version %d, firmware version %X\n",
			    tci->version.driver, tci->version.fpga);
	}
	if (tci->tcfd == -1)
		err = ENXIO;
	if (tci->iic.fd == -1)
		err = ENXIO;
	if (tci->iic_clk.fd == -1)
		err = ENXIO;
	return (err);
}

static int readdriftfile(struct timeCardInfo *tci)
{
	int err;
	FILE *driftf;
	float drift, aging;

	if (tcInfo.driftfname == NULL)
		return 1;

	driftf = fopen(tcInfo.driftfname, "r");
	if (driftf == NULL)
		return 1;
	err = fscanf(driftf, "%e %e", &drift, &aging);
	if (err == 2) {
		if ((aging < XO_AGING_MAX) && (aging > -XO_AGING_MAX) &&
		    (drift < XO_PULL_MAX) && (drift > -XO_PULL_MAX)) {
			tcInfo.dfaging = aging;
			tcInfo.dfoffset = drift;
			printf("driftfile drift %e ppu, aging %e ns/s\n", drift, aging);
		}
	}
	fclose(driftf);
	return 0;
}

static int captureTime(struct timeCardInfo *tci)
{
	int err;
	struct timecard_time gt;
	struct timecard_status gs;
	struct timespec target;

	err = ioctl(tci->tcfd, TCIOCGETTIME, (caddr_t)&gt);
	if (err) {
		perror("ioctl TCIOCGETTIME\n");
		exit (EIO);
	}
	err = ioctl(tci->tcfd, TCIOCGETSTATUS, (caddr_t)&gs);
	if (err) {
		perror("ioctl TCIOCGETSTATUS\n");
		exit (EIO);
	}
	tci->rcvTstmp = gt.kernel;
	tci->tcardClk = gt.card;
	tci->difftsc = gt.tsc - tci->tsc;
	tci->tsc = gt.tsc;
	tci->cpuid = gt.cpuid;

	tci->clk_clkstatus = gs.clk_status;
	tci->tod_status = gs.tod_status;
	tci->tod_gnss_status = gs.tod_gnss_status;
	tci->pps_slave_status = gs.pps_slave_status;

	tci->tod_utc_status = gs.tod_utc_status;
	if (tci->tod_utc_status & TC_TOD_UTC_STATUS_UTC_VALID)
		tci->tai_offset = (int)(tci->tod_utc_status & TC_TOD_UTC_STATUS_UTC_OFFSET_MASK);

	tci->clk_status_offset = gs.clk_offset;
	tci->clk_status_drift = gs.clk_drift;

	err = updateTimeStatus(tci);

	if (tci->status.time_valid && (tci->clk_clkstatus & TC_CLK_STATUS_IN_SYNC)) {
		tci->drift_acc_total += tci->clk_status_drift;
		tci->drift_acc_total_count++;
		tci->drift_acc += tci->clk_status_drift;
		tci->drift_acc_count++;

		tci->offset_acc_total += tci->clk_status_offset;
		tci->offset_acc_total_count++;
		tci->offset_acc += tci->clk_status_offset;
		tci->offset_acc_count++;
	}

	target = tci->tcardClk;
	target.tv_sec -= tci->tai_offset;
	tci->kernel_offset = timespecoffset(&target, &tci->rcvTstmp);
	tci->kernel_offset_acc += tci->kernel_offset;
	tci->precision = -25;	/* 2^precision. We are probably around 30ns. */
	return err;
}

static int pps_servo_status(struct timeCardInfo *tci)
{
	struct timecard_control tc;

	memset(&tc, 0, sizeof(struct timecard_control));
	tc.read = TC_CLK_SELECT | TC_PPS_SLAVE_CONTROL;
	ioctl(tci->tcfd, TCIOCCONTROL, &tc);
	tci->clk_pps_select = tc.clk_select;
	tci->pps_slave_control = tc.pps_slave_control;

	return 0;
}

static int pps_servo_enable(struct timeCardInfo *tci, int enable)
{
	int err;
	struct timecard_control tc;

	memset(&tc, 0, sizeof(struct timecard_control));
	if (enable) {
		tc.write = TC_CLK_SELECT | TC_PPS_SLAVE_CONTROL;
		tc.clk_select = 1;
		tc.pps_slave_control = 1;
	} else {
		tc.read = TC_CLK_SELECT | TC_PPS_SLAVE_CONTROL;
		tc.write = TC_CLK_CONTROL | TC_CLK_SELECT | TC_PPS_SLAVE_CONTROL;
		tc.clk_control = TC_CLK_CONTROL_DRIFT_ADJ | TC_CLK_CONTROL_ENABLE;
		tc.clk_select = 254;
		tc.pps_slave_control = 0;
	}
	err = ioctl(tci->tcfd, TCIOCCONTROL, &tc);
	tci->clk_pps_select = tc.clk_select;
	tci->pps_slave_control = tc.pps_slave_control;

	return err;
}

/*
 * It looks like occasionally as the GNSS (NEO-M9N) looses sync, it will
 * give an inaccurate Time Pulse, which causes the PPS Slave to give one
 * inaccurate update to the Adjustable Clock before going quiet until it
 * is in sync again. The Adjustable Clock will applying that drift value
 * until it receive s new updates from the PPS Slave, leading to wild
 * time and frequency moves.
 *
 * Attempt to disable the PPS Slave and select the Register adjustment
 * source, as soon as UTC_VALID is not set anymore.
 *
 * Output: tci->time_valid
 *
 * For observed cases see samples/extract.txt
 */
static int updateTimeStatus(struct timeCardInfo *tci)
{
	bool gnss_fix = false, inhold = false, insync = false, vitals_ok = false;
	int err = 0;
	struct timeCardStatus *st = &tci->status;
	struct timecard_time gt;

	if ((tci->tod_utc_status & TC_TOD_UTC_STATUS_UTC_VALID) &&
	    (tci->tod_gnss_status &  TC_TOD_GNSS_FIX_VALID) &&
	    (tci->tod_gnss_status & TC_TOD_GNSS_FIX_OK) &&
	    (tci->tai_offset > 32) && (tci->tai_offset < 42))
		gnss_fix = true;
	if (tci->clk_clkstatus & TC_CLK_STATUS_IN_HOLDOVER)
		inhold = true;
	if (!inhold && (tci->clk_clkstatus & TC_CLK_STATUS_IN_SYNC))
		insync = true;
	if (gnss_fix && insync)
		vitals_ok = true;

	switch (st->state) {
	case TC_START:
		pps_servo_status(tci);
		if (vitals_ok) {
			st->count = 10;
			st->state = TC_STEP;
			break;
		}
		st->state = TC_INIT;
		break;
	case TC_INIT:
		if (!gnss_fix)
			break;
		if (tci->pps_slave_control != 1)
			pps_servo_enable(tci, 1);
		if (!vitals_ok)
			break;
		st->count = 20;
		st->state = TC_STEP;
		break;
	case TC_STEP:
		if (st->count > 0) {
			st->count--;
			break;
		}
		if (tci->enable_kernel &&
		    (tci->status.kern_stepped == 0) &&
		    ((tci->kernel_offset > STEP_THRESH) ||
		    (tci->kernel_offset < -STEP_THRESH))) {
			err = ioctl(tci->tcfd, TCIOCGETTIME, (caddr_t)&gt);
			if (err) {
				printf("TC_STEP ioctl error %d\n", err);
				fflush(stdout);
			}
			gt.card.tv_sec -= tci->tai_offset;
			err = clock_settime(CLOCK_REALTIME, &gt.card);
			tci->status.kern_stepped = 1;
			err = 1; /* Give notice that time was stepped */
			printf("Time stepped offset %jd to %lu.%09lu\n", tci->kernel_offset,
			    gt.card.tv_sec, gt.card.tv_nsec);
			fflush(stdout);

			train_reset(tci);
		}
		st->count = 5;
		st->state = TC_PRESYNC;
		break;
	case TC_LOSTSYNC:
	case TC_LOSTFIX:
		if (vitals_ok) {
			/*
			 * Move the next aging calculation at least a day in the
			 * future after we lost fix/sync
			 */
			if (tci->nextaging < (tci->rcvTstmp.tv_sec + SECONDSPERDAY))
				tci->nextaging += SECONDSPERDAY;
			st->count = 3*60; /* Give FPGA servos time to settle */
			st->state = TC_PRESYNC;
			break;
		}
		if (gnss_fix) {
			if (tci->pps_slave_control != 1)
				pps_servo_enable(tci, 1);
			break;
		}
		/* XXX What now? */
		break;
	case TC_PRESYNC:
		if (vitals_ok) {
			if (st->count) {
				st->count--;
				break;
			}
			st->state = TC_SYNC;
			break;
		}
		if (tci->pps_slave_control == 1)
			pps_servo_enable(tci, 0);
		st->state = TC_LOSTSYNC;
		break;
	case TC_SYNC:
		if (vitals_ok) {
			if (st->time_valid == 0) {
				st->time_valid = 1;
				if (tcInfo.enable_training)
					train_reset(tci);
			}
			break;
		}
		/* If GNSS lost fix, but card still INSYNC, disable pps_slave,
		 * select REGISTERS as offset and drift source and clear the
		 * offset and drift.
		 */
		if (!gnss_fix && insync) {
			if (tci->pps_slave_control == 1)
				pps_servo_enable(tci, 0);
			st->time_valid = 0;
			st->state = TC_LOSTFIX;
			break;
		}

		if (tci->pps_slave_control == 1)
			pps_servo_enable(tci, 0);
		st->time_valid = 0;
		st->state = TC_LOSTSYNC;
		break;
	}
	return err;
}

static struct shmTime *getShmTime(int unit, int/*BOOL*/ forall)
{
	struct shmTime *p = NULL;

	int shmid;

	/* 0x4e545030 is NTP0.
	 * Big units will give non-ascii but that's OK
	 * as long as everybody does it the same way.
	 */
	shmid=shmget(0x4e545030 + unit, sizeof (struct shmTime),
		      IPC_CREAT | (forall ? 0666 : 0600));
	if (shmid == -1) { /* error */
		printf("SHM shmget (unit %d): %m", unit);
		return NULL;
	}
	p = (struct shmTime *)shmat (shmid, 0, 0);
	if (p == (struct shmTime *)-1) { /* error */
		printf("SHM shmat (unit %d): %m", unit);
		return NULL;
	}

	return p;
}

static inline void memory_barrier(void)
{
	atomic_thread_fence(memory_order_seq_cst);
}

static int updShmTime(struct timeCardInfo *tci)
{
	tci->shm->mode = 1;
	tci->shm->valid = 0;
	tci->shm->count++;
	memory_barrier();

	tci->shm->clockTimeStampSec = tci->tcardClk.tv_sec - tci->tai_offset;
	tci->shm->clockTimeStampUSec = tci->tcardClk.tv_nsec / 1000;
	tci->shm->clockTimeStampNSec = tci->tcardClk.tv_nsec;

	tci->shm->receiveTimeStampSec = tci->rcvTstmp.tv_sec;
	tci->shm->receiveTimeStampUSec = tci->rcvTstmp.tv_nsec / 1000;
	tci->shm->receiveTimeStampNSec = tci->rcvTstmp.tv_nsec;

	if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_VALID) {
		if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_ANNOUNCE) {
			if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_61)
				tci->shm->leap = LEAP_ADDSECOND;
			else if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_59)
				tci->shm->leap = LEAP_DELSECOND;
			else
				/* Can this happen? */
				tci->shm->leap = LEAP_NOWARNING;
		} else {
			tci->shm->leap = LEAP_NOWARNING;
		}
	} else {
		tci->shm->leap = LEAP_NOTINSYNC;
	}
	tci->shm->precision = tci->precision;
	memory_barrier();

	tci->shm->count++;
	memory_barrier();

	tci->shm->valid = 1;

	return 0;
}

static int clearKernTime(struct timeCardInfo *tc)
{
	struct timex tx;

	memset(&tx, 0, sizeof(tx));
	/* switch back to clkA */
	tx.modes = MOD_STATUS | MOD_CLKA;
	tx.status = 0;
	return ntp_adjtime(&tx);
}

/*
 * the main purpose of this is to do a 0 offset call, so the internal
 * timer is set to now, so that future calls with a non zero offset
 * does not go in the FLL mode.
 */
static int initKernTime(struct timeCardInfo *tci)
{
	struct timex tx;
	size_t len;
	uint64_t tcfreq;

	if (tci->kern_shift < MIN_KERN_SHIFT)
		tci->kern_shift = MIN_KERN_SHIFT;

	len = 8;
	if (sysctlbyname("kern.timecounter.tc.TimeCard.frequency",
	    &tcfreq, &len, NULL, 0) == -1) {
		perror("sysctl");
	}
	if (tcfreq == 1000000000UL)
		tci->kern_fudge_freq = (long)(FBSD_FREQ_ERR * SCALE_FREQ);
	else
		tci->kern_fudge_freq = 0;

	/* write everything zero and clear STA_PLL also in the process */
	memset(&tx, 0, sizeof(tx));
	tx.modes = MOD_NANO | MOD_STATUS | MOD_OFFSET | MOD_FREQUENCY | MOD_TIMECONST | MOD_CLKB;
	ntp_adjtime(&tx);

	memset(&tx, 0, sizeof(tx));
	tx.modes = MOD_NANO | MOD_STATUS | MOD_OFFSET | MOD_FREQUENCY | MOD_TIMECONST | MOD_CLKB;
	tx.freq = (long)tci->kern_fudge_freq;
	tx.status = STA_PLL;
	return ntp_adjtime(&tx);
}

static int updKernTime(struct timeCardInfo *tci)
{
	int err;
	int64_t freq, offset;
	struct timex tx;

	memset(&tx, 0, sizeof(tx));
	if ((tci->rcvTstmp.tv_sec - tci->kernel_upd_tstmp) < 1 << tci->kern_shift) {
		tx.modes = MOD_NANO | MOD_CLKB;
		err = ntp_adjtime(&tx);
		tci->ntpa_offset = tx.offset;
		tci->ntpa_freq = tx.freq;
		tci->ntpa_status = tx.status;
		if (tci->kern_shift && ((tci->rcvTstmp.tv_sec - tci->kernel_upd_tstmp) < 1 << (tci->kern_shift - 1)))
			tci->kernel_offset_acc = 0;
		return 0;
	}

	offset = tci->kernel_offset_acc;
	freq = offset;
	/* original scaling from ns/s to ppm -> freq = (nsps << 16) / 1000LL */
	freq <<= 16 - (tci->kern_shift + 1);
	if (freq > (MAXFREQ << 16))
		freq = (MAXFREQ << 16);
	if (freq < -(MAXFREQ << 16))
		freq = -(MAXFREQ << 16);
	freq += 500;
	freq /= 1000;

	if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_VALID) {
		if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_ANNOUNCE) {
			if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_61)
				tx.status |= STA_INS;
			else if (tci->tod_utc_status & TC_TOD_UTC_STATUS_LEAP_59)
				tx.status |= STA_DEL;
		}
	}
	if (offset == 0) {
		tci->kern_shift_stbl++;
		if (tci->kern_shift_stbl > MAX_KERN_STABLE) {
			tci->kern_shift_stbl = MAX_KERN_STABLE;
			if (tci->kern_shift < MAX_KERN_SHIFT) {
				tci->kern_shift++;
				tci->kern_shift_stbl = 0;
			}
		}
	} else {
		tci->kern_shift_stbl--;
		if (tci->kern_shift_stbl < -MAX_KERN_STABLE) {
			tci->kern_shift_stbl = -MAX_KERN_STABLE;
			if (tci->kern_shift > MIN_KERN_SHIFT) {
				tci->kern_shift--;
				tci->kern_shift_stbl = 0;
			}
		}
	}

	tx.modes = MOD_NANO | MOD_STATUS | MOD_MAXERROR | MOD_ESTERROR |
	    MOD_TIMECONST | MOD_CLKB;
	tx.status = STA_PLL;
	tx.constant = tci->kern_shift;
	tx.modes |= MOD_OFFSET;
	/* Handle the case of the kernel module running at 1GHz */
	if (freq == 0 && tci->kern_fudge_freq != 0)
		tx.freq = tci->kern_fudge_freq;
	else
		tx.freq = freq;
	tx.modes |= MOD_FREQUENCY;
	tci->kernel_upd_tstmp = tci->rcvTstmp.tv_sec;
	err = ntp_adjtime(&tx);
	tci->ntpa_offset = tx.offset;
	tci->ntpa_freq = tx.freq;
	tci->ntpa_status = tx.status;
	tci->kernel_offset_acc = 0;
	if (err < 0 || err > 5)
		return err;
	return 0;
}

int64_t timespecoffset(struct timespec *ts1, struct timespec *ts2)
{
	int64_t offs = 0L;

	offs = ts1->tv_sec;
	offs -= ts2->tv_sec;
	offs *= NANOSECOND;
	offs += ts1->tv_nsec;
	offs -= ts2->tv_nsec;

	return offs;
}

static int
axi_iic_xfer(struct axi_iic_info *iic, struct iic_msg *msgs, uint32_t num)
{
	struct iic_rdwr_data rdwr_data = { msgs, num };

	return ioctl(iic->fd, I2CRDWR, &rdwr_data);
}

int rdI2Creg(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, uint8_t *rxbuf, uint8_t rxlen)
{
	struct iic_msg msgs[2];
	uint8_t txbuf[8];
	int err;

	msgs[0].slave = (addr << 1) | IIC_M_WR;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	txbuf[0] = reg;
	msgs[0].buf = txbuf;

	msgs[1].slave = (addr << 1) | IIC_M_RD;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = rxlen;
	msgs[1].buf = rxbuf;

	err = axi_iic_xfer(iic, msgs, 2);
	if (err)
		iic->iicxfererrcnt++;
	return (err);
}

static uint32_t rdI2Creg32(struct axi_iic_info *iic, uint8_t addr, uint8_t reg)
{
	struct iic_msg msgs[2];
	uint8_t txbuf[8], rxbuf[8];
	uint32_t *rval;
	int err;

	msgs[0].slave = (addr << 1) | IIC_M_WR;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = txbuf;
	txbuf[0] = reg;

	msgs[1].slave = (addr << 1) | IIC_M_RD;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = 4;
	msgs[1].buf = rxbuf;
	rxbuf[0] = 0;
	rxbuf[1] = 0;
	rxbuf[2] = 0;
	rxbuf[3] = 0;
	rxbuf[4] = 0;

	err = axi_iic_xfer(iic, msgs, 2);
	if (err)
		iic->iicxfererrcnt++;
	rval = (uint32_t *)rxbuf;
	if ((rxbuf[1] == 0xff) && (rxbuf[2] == 0xff) && (rxbuf[3] == 0xff))
		iic->iicffcnt++;
	return (*rval);
}

static uint32_t rdI2Creg32FF(struct axi_iic_info *iic, uint8_t addr, uint8_t reg)
{
	uint32_t val;
	int count;
	struct timespec tsleep;

	for (count = 0; count < 4; count++) {
		val = rdI2Creg32(iic, addr, reg);
		if ((val & 0xffffff00) != 0xffffff00)
			break;
		/* Try something radical */
		if (count > 1) {
			rdI2Creg32(iic, 1, XO_STATUS);
			if (count > 2) {
				tsleep.tv_sec = 0;
				tsleep.tv_nsec = 10000000;
				nanosleep(&tsleep, NULL);
			}
			continue;
		}
		rdI2Creg32(iic, addr, XO_STATUS);
	}
	return val;
}

/* Reliable version */
static uint32_t rdI2Creg32R(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, uint32_t prev)
{
	uint32_t val1, val2;
	int count = 10;
	do {
		val1 = rdI2Creg32FF(iic, addr, reg);
		val2 = rdI2Creg32FF(iic, addr, reg);
		if (val1 == val2)
			return (val1);
	} while (--count);
	return (prev);
}

#if 0
static float rdI2Cfloat(struct axi_iic_info *iic, uint8_t addr, uint8_t reg)
{
	uint32_t rval;
	volatile float *fvalp;

	rval = rdI2Creg32(iic, addr, reg);
	fvalp = (float *)&rval;
	return (*fvalp);
}
#endif

static float rdI2CfloatFF(struct axi_iic_info *iic, uint8_t addr, uint8_t reg)
{
	uint32_t rval;
	volatile float *fvalp;

	rval = rdI2Creg32FF(iic, addr, reg);
	fvalp = (float *)&rval;
	return (*fvalp);
}

/*
 * Do two reads of the same register and check if the same. If not,
 * retry a few times.
 *
 * If dev is not 0, allow that much change.
 *
 * Return val2 on success, otherwise return prev.
 *
 * NOTE: Do not use for fast changing registers like, Heater Power
 * without dev, or use the MM version..
 */
static float rdI2CfloatR(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, float prev, float dev)
{
	float val1, val2;
	int count = 3;
	do {
		val1 = rdI2CfloatFF(iic,  addr, reg);
		val2 = rdI2CfloatFF(iic,  addr, reg);
		if (val1 == val2)
			return (val2);
		if (dev != 0.0) {
			if (val2 < val1) {
				if (val2 > (val1 - dev))
					return (val2);
			} else {
				if (val2 < (val1 + dev))
					return (val2);
			}
		}
		/* Sleep a little if we had an error */
		usleep(1000);
		iic->iicerrcnt++;
	} while (--count);
	return (prev);
}

/* Read float with a min / max bound. Retry if outside. */
static float rdI2CfloatMM(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, float prev, float minval, float maxval)
{
	float val;
	int count = 3;
	do {
		val = rdI2CfloatFF(iic,  addr, reg);
		if ((val > minval) && (val < maxval))
			return (val);
		/* Sleep a little if we had an error */
		usleep(1000);
		iic->iicerrcnt++;
	} while (--count);
	return (prev);
}

static int wrI2Creg32(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, uint32_t val)
{
	struct iic_msg msgs[1];
	uint8_t txbuf[8];
	int err;

	msgs[0].slave = (addr << 1) | IIC_M_WR;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 5;
	msgs[0].buf = txbuf;
	txbuf[0] = reg;
	txbuf[1] = val & 0xff;
	txbuf[2] = (val >> 8) & 0xff;
	txbuf[3] = (val >> 16) & 0xff;
	txbuf[4] = (val >> 24) & 0xff;
	err = axi_iic_xfer(iic, msgs, 1);
	if (err)
		iic->iicxfererrcnt++;
	return (err);
}

static int wrI2CfloatR(struct axi_iic_info *iic, uint8_t addr, uint8_t reg, float val)
{
	uint32_t *valp;
	float fval2;
	int count = 3;
	do {
		valp = (uint32_t *)&val;
		wrI2Creg32(iic, addr, reg, *valp);
		fval2 = rdI2CfloatR(iic, addr, reg, 0.0, 0.0);
		if (fval2 == val)
			return (0);
	} while (--count);
	return (1);
}

/*
 * TC 1692299267.273176754 OS 1692299230.272996813 offset drift
 * Sts TODsts GNSSsts UTCsts XO offset temp voltage watt status
 */
static int logstats(struct timeCardInfo *tci)
{
	if (need_freopen) {
		fclose(tci->logf);
		tci->logf = fopen(tci->logfname, "a");
	}
	if (tci->logf == NULL)
		return 0;

	fprintf(tci->logf, "TC %lu.%09lu OS %lu.%09lu ", tci->tcardClk.tv_sec,
	    tci->tcardClk.tv_nsec, tci->rcvTstmp.tv_sec, tci->rcvTstmp.tv_nsec);
	fprintf(tci->logf, "% 3d % 3d", tci->clk_status_offset,
	    tci->clk_status_drift);
	fprintf(tci->logf, " 0x%X 0x%X 0x%X 0x%X", tci->clk_clkstatus,
	    tci->tod_status, tci->tod_gnss_status, tci->tod_utc_status);
	fprintf(tci->logf, " 0x%X %d %d", tci->pps_slave_status,
	    tci->status.state, tci->status.time_valid);
	fprintf(tci->logf, " K % 2jd % 2ld", tci->kernel_offset,
	    tci->ntpa_offset);
	fprintf(tci->logf, " %.4g %d", FREQTOPPM(tci->ntpa_freq),
	    tci->kern_shift);
#ifdef SHOW_TSC
	fprintf(tci->logf, " %ju %2d", tci->difftsc, tci->cpuid);
#endif
	if (tci->enable_clock)
		fprintf(tci->logf, " XO %.5e %.4f %.5e %u %u",
		    tci->xo_offset, tci->xo_power,
		    tci->temp_comp,
		    tci->iic_clk.iicerrcnt, tci->iic_clk.iicffcnt);
#ifdef USE_BME
	if (tci->enable_bme)
		fprintf(tci->logf, " BME %0.2f %0.2f %0.2f",
		    tci->bme_temperature,
		    tci->bme_pressure,
		    tci->bme_humidity);
#endif
	fprintf(tci->logf, " TR %d %d %d %d %e %d %d %e %e",
	    tci->offset_acc_total,
	    tci->offset_acc_total_count,
	    tci->offset_acc,
	    tci->offset_acc_count,
	    tci->train_adj,
	    tci->train_shift,
	    tci->train_stable,
	    tci->aging,
	    tci->xo_pull);
	fprintf(tci->logf, "\n");
	fflush(tci->logf);
	return 0;
}

/*
 * prime the system
 */
static int temp_comp_init(struct timeCardInfo *tci)
{
	int retv;
	if ((tci->xo_power < tci->xo_power_min) ||
	    (tci->xo_power > tci->xo_power_max)) {
		if (tci->xo_power_ref != 0.0) {
			tci->xo_power = tci->xo_power_ref;
		} else {
			tci->temp_comp = 0.0;
			return 0;
		}
	}

	tci->xo_power_cnt = TEMP_COMP_CNT - 1;
	tci->xo_power_acc = tci->xo_power * tci->xo_power_cnt;
	retv =  temp_comp(tci);
	printf("temp_comp_init, xo_power %.4f, comp %.5e\n",
	    tci->xo_power, tci->temp_comp);
	return retv;
}

static int temp_comp(struct timeCardInfo *tci)
{
	float comp;
	int i;

	if ((tci->xo_power < tci->xo_power_min) ||
	    (tci->xo_power > tci->xo_power_max)) {
		tci->temp_comp = 0.0;
		return 0;
	}
	tci->xo_power_acc += tci->xo_power;
	tci->xo_power_cnt++;
	if (tci->xo_power_cnt < TEMP_COMP_CNT)
		return 0;
	comp = tci->xo_power_acc / tci->xo_power_cnt;
	tci->xo_power_acc = 0;
	tci->xo_power_cnt = 0;

	comp -= tci->xo_power_ref;
	comp *= tci->temp_comp_fact;
	comp *= 1.0E-9;
	tci->temp_comp = comp;
	return 0;
}

static void timespec_sub(struct timespec *_v1, struct timespec *_v2,
    struct timespec *_result)
{
	if (_v1->tv_nsec < _v2->tv_nsec) {
		_result->tv_nsec = 1000000000 + _v1->tv_nsec - _v2->tv_nsec;
		_result->tv_sec = _v1->tv_sec - _v2->tv_sec - 1;
	} else {
		_result->tv_nsec = _v1->tv_nsec - _v2->tv_nsec;
		_result->tv_sec = _v1->tv_sec - _v2->tv_sec;
	}
}

static int train_init(struct timeCardInfo *tci, int hotstart)
{
	int cold = 0;

	if (tci->train_period == 0)
		tci->train_period = TRAIN_PERIOD;
	if (tci->train_stable_val == 0)
		tci->train_stable_val = TRAIN_STABLE;
	tci->train_stable = 0;
	if (tci->train_stable_max == 0)
		tci->train_stable_max = MAX_TRAIN_STABLE;
	if (tci->train_shift_max == 0)
		tci->train_shift_max = MAX_TRAIN_SHIFT;
	/* XXX Is checking for 0 the best way? */
	if (tci->train_shift_min == 0)
		tci->train_shift_min = MIN_TRAIN_SHIFT;
	tci->train_shift = MIN_TRAIN_SHIFT + 1;
	if (tci->train_shift < tci->train_shift_min)
		tci->train_shift = tci->train_shift_min;
	if (tci->train_shift > tci->train_shift_max)
		tci->train_shift = tci->train_shift_max;

	/* Probably a cold start, but with a driftfile */
	if (tci->xo_offset == 0.0 && (tci->dfaging != 0.0 || tci->dfoffset != 0.0)) {
		tci->aging = tci->dfaging;
		tci->train_pull = tci->dfoffset;
		xo_update(tci);
		cold = 1;
	/* Warm start, assume the values in the XO is more correct. */
	} else if (tci->xo_offset != 0.0) {
		tci->aging = tci->xo_aging;
		/* Can happen if we move from manual aging */
		if ((tci->xo_aging == 0.0) && (tci->dfoffset != 0.0))
			tci->aging = tci->dfaging;
		tci->train_pull = tci->xo_pull;
	}
	if (tcInfo.enable_clock_temp_comp && tci->train_pull != 0.0)
		tci->train_pull -= tci->temp_comp;
	if (tci->aging != 0.0 || tci->train_pull != 0.0)
		printf("Aging %e ns/s, pull %e s, %s start\n",
		    tci->aging, tci->train_pull, cold ? "cold": "warm");

	return 0;
}

/*
 * Used to clear the training info. For instance when the card
 * is not in sync yet.
 */
static int train_reset(struct timeCardInfo *tci)
{
	tci->drift_acc_total = 0;
	tci->drift_acc_total_count = 0;
	tci->drift_acc_count = 0;
	tci->drift_acc = 0;

	tci->offset_acc_total = 0;
	tci->offset_acc_total_count = 0;
	tci->offset_acc_count = 0;
	tci->offset_acc = 0;

	tci->nexttrain = tci->rcvTstmp.tv_sec + tci->train_period;

	return 0;
}

static int train(struct timeCardInfo *tci)
{
	int32_t offset = 0, offset_lt = 0;

	if (tci->nexttrain > tci->rcvTstmp.tv_sec)
		return 0;

	if (tci->train_use_total_offset) {
		offset = tci->offset_acc_total;
	} else {
		if (tci->offset_acc_total >= 0)
			offset_lt = (tci->offset_acc_total + (1 << (TRAIN_TOTAL_FRAC - 1))) >> TRAIN_TOTAL_FRAC;
		else
			offset_lt = (tci->offset_acc_total - (1 << (TRAIN_TOTAL_FRAC - 1))) >> TRAIN_TOTAL_FRAC;
		offset = tci->offset_acc;
	}
	if ((offset < tci->train_stable_val) && (offset > -tci->train_stable_val))
		tci->train_stable++;
	else if ((offset > ((tci->train_stable_val * 3) / 2)) || (offset < -((tci->train_stable_val * 3) / 2)))
		tci->train_stable = -tci->train_stable_max - 1;
	else
			tci->train_stable--;

	if (tci->train_stable > tci->train_stable_max) {
		tci->train_stable = tci->train_stable_max;
		if (tci->train_shift < tci->train_shift_max) {
			tci->train_shift++;
			tci->train_stable = 0;
		}
	}
	if (tci->train_stable < -tci->train_stable_max) {
		tci->train_stable = -tci->train_stable_max;
		if (tci->train_shift > tci->train_shift_min) {
			tci->train_shift--;
			tci->train_stable = 0;
		}
	}

	if (tci->train_use_total_offset) {
		tci->train_adj = offset;
		tci->train_adj /= tci->train_period << tci->train_shift;
	} else {
		tci->train_adj = offset;
		tci->train_adj /= tci->train_period << tci->train_shift;
		tci->train_adj += (float)offset_lt / (tci->train_period << (tci->train_shift / 2));
	}

	tci->train_adj *= 1.0E-9;
	tci->train_pull += tci->train_adj;

	tci->drift_acc_count = 0;
	tci->drift_acc = 0;
	tci->offset_acc_count = 0;
	tci->offset_acc = 0;

	tci->nexttrain = tci->rcvTstmp.tv_sec + tci->train_period;

	return 1;
}

/*
 * 0x50 - EEPROM
 * 0x58 - Serial
 * use the EUI-48 number as the serial number.
 */
static int probe_eeprom(struct timeCardInfo *tci)
{
	int err;
	struct axi_iic_info *iic = &tci->iic;
	
	/* Read the EUI-48 number */
	err = rdI2Creg(iic, 0x58, 0x9A, tci->serialno, 6);
	if (err)
		printf("rdI2Creg returned err %X\n", err);
	sprintf(tci->serialnotxt, "%02X:%02X:%02X:%02X:%02X:%02X",
	    tci->serialno[0], tci->serialno[1], tci->serialno[2],
	    tci->serialno[3], tci->serialno[4], tci->serialno[5]);
	return err;
}

#ifdef USE_BME
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
	int err;
	struct identifier id;

	id = *((struct identifier *)intf_ptr);
	err = rdI2Creg(id.iic, id.dev_addr, reg_addr, data, len);
	if (err) {
		return BME280_E_COMM_FAIL;
	}
	return BME280_OK;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
	usleep(period);
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
	int err;
	struct identifier id;
	struct iic_msg msg;

	id = *((struct identifier *)intf_ptr);

	uint8_t *buf = malloc((2 + len) * sizeof(uint8_t));

	if (buf == NULL) {
		return BME280_E_COMM_FAIL;
	}

	buf[0] = reg_addr;

	for (uint8_t i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}

	msg.slave = id.dev_addr << 1 | IIC_M_WR;
	msg.flags = IIC_M_WR;
	msg.len = 1 + len;
	msg.buf = buf;

	err = axi_iic_xfer(id.iic, &msg, 1);

	if (err) {
		free(buf);
		return BME280_E_COMM_FAIL;
	}

	free(buf);
	return BME280_OK;
}

static int probe_bme280(struct timeCardInfo *tci)
{
	int8_t rslt;
	struct bme280_settings settings = {0};
	
	bmeid.dev_addr = BME280_I2C_ADDR_PRIM;
	bmeid.iic = &tci->iic;
	bmedev.intf = BME280_I2C_INTF;
	bmedev.read = user_i2c_read;
	bmedev.write = user_i2c_write;
	bmedev.delay_us = user_delay_us;

	/* Update interface pointer with the structure that contains both device address and file descriptor */
	bmedev.intf_ptr = &bmeid;
	
	rslt = bme280_init(&bmedev);
	if (rslt != BME280_OK) {
		fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
		exit(1);
	}

	/* Get the current sensor settings */
	rslt = bme280_get_sensor_settings(&settings, &bmedev);
	if (rslt != BME280_OK) {
		fprintf(stderr, "Failed to get sensor settings (code %+d).", rslt);
		return rslt;
	}

	/* Trying normal mode a 1 per second */
	settings.filter = BME280_FILTER_COEFF_2;
	settings.osr_h = BME280_OVERSAMPLING_1X;
	settings.osr_p = BME280_OVERSAMPLING_1X;
	settings.osr_t = BME280_OVERSAMPLING_1X;
	settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	/* Set the sensor settings */
	rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bmedev);
	if (rslt != BME280_OK) {
		fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
		return rslt;
	}
	rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bmedev);
	if (rslt != BME280_OK) {
		fprintf(stderr, "Failed to set power mode (code %+d).", rslt);
		return rslt;
	}

	return rslt;
}

static int read_bme280(struct timeCardInfo *tci)
{
	int8_t rslt;
	struct bme280_data comp_data;
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bmedev);
	tci->bme_temperature = comp_data.temperature;
	tci->bme_pressure = comp_data.pressure * 0.01;
	tci->bme_humidity = comp_data.humidity;

	return rslt;
}
#endif


/*
 * Initialize XO.
 *
 * Mostly reading the current values, so that train_init() can make
 * a decision about what to use.
 */
static int xo_init(struct timeCardInfo *tci)
{
	float xo_pull;
	float xo_offset;
	float xo_aging;
	int err;
	uint8_t text[256];
	struct axi_iic_info *iic = &tci->iic_clk;

	memset(text, 0, 256);
	err = rdI2Creg(iic, SIT_ADDR, XO_PART_NUMBER, text, 255);
	printf("%s\n", text);
	memset(text, 0, 256);
	err = rdI2Creg(iic, SIT_ADDR, XO_NOM_FREQ, text, 32);
	printf("%s ", text);
	memset(text, 0, 256);
	err = rdI2Creg(iic, SIT_ADDR, XO_SERIAL_NUMBER, text, 32);
	printf("%s ", text);
	memset(text, 0, 256);
	err = rdI2Creg(iic, SIT_ADDR, XO_FAB_DATE, text, 32);
	printf("%s\n", text);
	
	xo_pull = rdI2CfloatR(iic, SIT_ADDR, XO_PULL_VALUE, 0, 0.0);
	xo_aging = rdI2CfloatR(iic, SIT_ADDR, XO_AGE_COMP, 0, 0.0);
	xo_offset = rdI2CfloatR(iic, SIT_ADDR, XO_OFFSET, 0, 1.0E-12);

	tci->xo_pull = xo_pull;
	tci->xo_aging = xo_aging;
	tci->xo_offset = xo_offset;

	return err;
}

/*
 * update XO with latest values.
 *
 * updates XO_PULL_VALUE and XO_AGE_COMP
 */
static int xo_update(struct timeCardInfo *tci)
{
	int err = 0;
	float total_pull = tci->temp_comp + tci->train_pull;

	struct axi_iic_info *iic = &tci->iic_clk;
	if (tci->xo_aging != tci->aging) {
		wrI2CfloatR(iic, SIT_ADDR, XO_AGE_COMP, tci->aging);
		tci->xo_aging = rdI2CfloatR(iic, SIT_ADDR, XO_AGE_COMP, tci->aging, 0.0);
		if (tci->xo_aging != tci->aging) {
			/* XXX What should be done? */
			printf("xo_update: XO_AGE_COMP, expected %e, read %e\n", tci->aging, tci->xo_aging);
		}
	}
	if (tci->xo_pull != total_pull) {
		wrI2CfloatR(iic, SIT_ADDR, XO_PULL_VALUE, total_pull);
		tci->xo_pull = rdI2CfloatR(iic, SIT_ADDR, XO_PULL_VALUE, total_pull, 0.0);
		if (tci->xo_pull != total_pull) {
			/* XXX What should be done? */
			printf("xo_update: XO_PULL_VALUE, expected %e, read %e\n", total_pull, tci->xo_pull);
		}
	}
	return err;
}

/*
 * Capture stats from the xo.
 */
static int xo_stats(struct timeCardInfo *tci)
{
	struct axi_iic_info *iic = &tci->iic_clk;

	/* Read from sitime, use Reliable versions */
	tci->xo_offset = rdI2CfloatR(iic, SIT_ADDR, XO_OFFSET, tci->xo_offset, 10.0e-14);
	tci->xo_power = rdI2CfloatMM(iic, SIT_ADDR, XO_HEATER_POWER, tci->xo_power, -1.0, 10.0);

	return 0;
}

static void pullstatsadd(struct timeCardInfo *tci)
{
	float adj;

	/*
	 * Use xo_offset because that includes the aging the XO applied
	 * and add tci->train_adj because it has not been applied yet,
	 * but will be, just after this.
	 */
	pullbuf[tci->pullbindx] = tci->xo_offset + tci->train_adj;
	pulltsbuf[tci->pullbindx] = tci->rcvTstmp.tv_sec;

	if (tci->pullbcnt < PULLBUFSIZ)
		tci->pullbcnt++;
	tci->pullbindx++;
	if (tci->pullbindx >= PULLBUFSIZ)
		tci->pullbindx = 0;
}

/*
 * _off
 *  0 - last indx
 * -1 - before last indx
 */
static int32_t pullstatindx(struct timeCardInfo *tci, int32_t _off)
{
	int32_t indx;
	indx = tci->pullbindx;
	indx -= 1;	/* bindx point to next spot to be used */
	indx += _off;
	if (indx < 0)
		indx += PULLBUFSIZ;
	return indx;
}

/*
 * We are called once per second after train().
 */
static void calcaging(struct timeCardInfo *tci)
{
	float lastpull, prevpull, taging;
	time_t laststmp, prevstmp, tperiod;
	int32_t div, driftoff,  indx;

	if (tci->nextaging > tci->rcvTstmp.tv_sec)
		return;
	pullstatsadd(tci);
	if (tci->pullbcnt == 1) {
		tci->nextaging += SECONDSPERDAY;
		printf("First pull sample %lu\n", tci->rcvTstmp.tv_sec);
		fflush(stdout);
		return;
	}
	indx = pullstatindx(tci, 0);
	lastpull = pullbuf[indx];
	laststmp = pulltsbuf[indx];
	indx = pullstatindx(tci, -1);
	prevpull = pullbuf[indx];
	prevstmp = pulltsbuf[indx];

	tperiod = laststmp - prevstmp;
	taging = lastpull - prevpull;
	taging /= tperiod;
	div = tci->pullbcnt;
	if (div > 80)
		div = 80;
	div /= 2;
	if (div == 0)
		div = 1;
	/* prime the first time */
	if (tci->aging == 0.0)
		tci->aging = taging;
	tci->aging += ((taging - tci->aging) / div);
	tci->nextaging += SECONDSPERDAY;
	printf("Aging %e ns/s, avg %e, period %lu s, total over period %e, tadj %e\n",
		    taging, tci->aging, tperiod, lastpull - prevpull, tci->train_adj);
	fflush(stdout);
	return;
}

static void driftfupdate(struct timeCardInfo *tci)
{
	FILE *driftf;

	if (tci->nextdriftf > tci->rcvTstmp.tv_sec)
		return;
	if (tci->driftfname == NULL)
		return;
	tci->nextdriftf += (60 * 60);

	if ((tci->dfaging == tci->aging) && (tci->dfoffset == tci->xo_offset))
		return;
	tci->dfaging = tci->aging;
	tci->dfoffset = tci->xo_offset;

	driftf = fopen(tci->driftfname, "w");
	if (driftf == NULL) {
		printf("Could not open driftfile %s\n", tci->driftfname);
		return;
	}
	fprintf(driftf, "%e %e\n", tci->dfoffset, tci->dfaging);
	fclose(driftf);
}
