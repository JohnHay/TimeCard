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

#ifndef _TIMECARD_H_
#define _TIMECARD_H_

/*
 * IOCTLs needed
 *
 * Reading:
 * core and firmware versions
 * selections: gnss, pps, baud rate, clk select source for adjustments
 * time to leap, uart polarity, leap correction
 * eeprom serial number?
 *
 * Configure:
 * TOD protocol select, gnss, messages
 * TOD Baud rate
 * uart polarity, leap correction
 * clk select source for adjustments
 *
 * reset - gnss1, gnss2, ?
 *
 * clear TOD status errs
 *
 * Cable delay
 * ?
 */

/* Return the card time and kernel time */
struct timecard_time {
	struct timespec card;
	struct timespec kernel;
	uint64_t tsc;
	u_int cpuid;
};

/* Return various status registers */
struct timecard_status {
	uint32_t clk_status;
	int32_t clk_offset;
	int32_t clk_drift;
	uint32_t tod_status;
	uint32_t tod_utc_status;
	uint32_t tod_gnss_status;
	uint32_t tod_sat_num;
	uint32_t pps_slave_status;
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

struct timecard_control {
	uint32_t read;
	uint32_t write;
	uint32_t clk_control;
	uint32_t clk_select;
	uint32_t clk_time_adj_nsec;
	uint32_t clk_time_adj_sec;
	uint32_t clk_offset_adj_value;
	uint32_t clk_offset_adj_interval;
	uint32_t clk_drift_adj_value;
	uint32_t clk_drift_adj_interval;
	uint32_t clk_insync_threshold;
	uint32_t clk_servo_offset_Kp;
	uint32_t clk_servo_offset_Ki;
	uint32_t clk_servo_drift_Kp;
	uint32_t clk_servo_drift_Ki;
	uint32_t pps_slave_control;
	uint32_t pps_slave_cable_delay;
	uint32_t tod_control;
	uint32_t tod_uart_baud_rate;
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
	uint32_t axi_hxicap;
	uint32_t axi_qspi;
};

#define TCIOCGETTIME	_IOR('t', 1, struct timecard_time)
#define TCIOCGETSTATUS	_IOR('t', 2, struct timecard_status)
#define TCIOCGETVERSION	_IOR('t', 3, struct timecard_version)
#define TCIOCCONTROL	_IOWR('t', 4, struct timecard_control)

#endif /* _TIMECARD_H_ */
