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

#ifndef _TIMECARD_H_
#define _TIMECARD_H_

/*
 * Outstanding:
 *
 * Configure:
 * TOD uart polarity
 * TOD leap correction
 * AXIGPIOExtGPIO2 - MAC/Clock COM select + EEPROM WP
 * AXIGPIOGNSSGPIO2 - reset - gnss1, gnss2
 *
 */

/* Return the card time and kernel time */
struct timecard_time {
	struct timespec card;	/* time read from card */
	struct timespec kernel;	/* kernel timestamp */
	uint64_t tsc;		/* TSC timestamp */
	u_int cpuid;		/* CPU ID on which it happened */
};

/* Return various status registers */
struct timecard_status {
	uint32_t clk_status;		/* ClockStatus */
	int32_t clk_offset;		/* ClockStatusOffset - two's compliment */
	int32_t clk_drift;		/* ClockStatusDrift - two's compliment */
	uint32_t tod_status;		/* TodSlaveStatus */
	uint32_t tod_utc_status;	/* TodSlaveUtcStatus */
	uint32_t tod_gnss_status;	/* TodSlaveAntennaStatus */
	uint32_t tod_sat_num;		/* TodSlaveSateliteNumber */
	int32_t tod_time_to_leap_sec;	/* TodSlaveTimeToLeapSecond */
	uint32_t pps_slave_status;	/* PpsSlaveStatus */
};

#define TC_CLK_CONTROL			(1 << 0)
#define TC_CLK_SELECT			(1 << 1)
#define TC_CLK_TIME_ADJ			(1 << 2)
#define TC_CLK_OFFSET_ADJ		(1 << 3)
#define TC_CLK_DRIFT_ADJ		(1 << 4)
#define TC_CLK_INSYNC_THRESH		(1 << 5)
#define TC_CLK_SERVO_ADJ		(1 << 6)
#define TC_PPS_SLAVE_CONTROL		(1 << 7)
#define TC_PPS_SLAVE_STATUS_CLR		(1 << 8)
#define TC_PPS_SLAVE_CABLE_DELAY	(1 << 9)
#define TC_TOD_CONTROL			(1 << 10)
#define TC_TOD_STATUS_CLR		(1 << 11)
#define TC_TOD_UART_BAUD_RATE		(1 << 12)
#define TC_TOD_UART_POLARITY		(1 << 13)
#define TC_TOD_CORRECTION		(1 << 14)
#define TC_GPIO_EXT_GPIO2		(1 << 15)
#define TC_GPIO_GNSS_RESET		(1 << 16)

struct timecard_control {
	uint32_t read;
	uint32_t write;
	uint32_t clk_control;			/* ClockControl */
	uint32_t clk_select;			/* ClockSelect */
	uint32_t clk_time_adj_nsec;		/* ClockTimeAdjValueL */
	uint32_t clk_time_adj_sec;		/* ClockTimeAdjValueH */
	uint32_t clk_offset_adj_value;		/* ClockOffsetAdjValue */
	uint32_t clk_offset_adj_interval;	/* ClockOffsetAdjInterval */
	uint32_t clk_drift_adj_value;		/* ClockDriftAdjValue */
	uint32_t clk_drift_adj_interval;	/* ClockDriftAdjInterval */
	uint32_t clk_insync_threshold;		/* ClockInSyncThreshold */
	uint32_t clk_servo_offset_Kp;		/* ClockServoOffsetFactorP */
	uint32_t clk_servo_offset_Ki;		/* ClockServoOffsetFactorI */
	uint32_t clk_servo_drift_Kp;		/* ClockServoDriftFactorP */
	uint32_t clk_servo_drift_Ki;		/* ClockServoDriftFactorI */
	uint32_t pps_slave_control;		/* PpsSlaveControl */
	uint32_t pps_slave_cable_delay;		/* PpsSlaveCableDelay */
	uint32_t tod_control;			/* TodSlaveControl */
	uint32_t tod_uart_baud_rate;		/* TodSlaveUartBaudRate */
	uint32_t tod_uart_polarity;		/* TodSlaveUartPolarity */
	uint32_t tod_correction;		/* TodSlaveCorrection */
	uint32_t gpio_ext_gpio2;		/* AXIGPIOextGPIO2 */
	uint32_t gpio_gnss_reset;		/* AXIGPIOGNSSGPIO2 */
};

struct timecard_version {
	uint32_t length;
	uint32_t driver;
	uint32_t fpga;
	uint32_t core_list;
	uint32_t adj_clock;
	uint32_t sig_gen;
	uint32_t sig_tstamper;
	uint32_t pps_gen;
	uint32_t freq_counter;
	uint32_t clock_detector;
	uint32_t sma_selector;
	uint32_t pps_source_selector;
	uint32_t pps_slave;
	uint32_t tod_slave;
	uint32_t axi_pcie;
	uint32_t axi_gpio;
	uint32_t axi_iic;
	uint32_t axi_uart;
	uint32_t axi_hwicap;
	uint32_t axi_qspi;
};

#define TCIOCGETTIME	_IOR('t', 1, struct timecard_time)
#define TCIOCGETSTATUS	_IOR('t', 2, struct timecard_status)
#define TCIOCGETVERSION	_IOR('t', 3, struct timecard_version)
#define TCIOCCONTROL	_IOWR('t', 4, struct timecard_control)

#endif /* _TIMECARD_H_ */
