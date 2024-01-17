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

#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <timecard.h>
#include <timecard_reg.h>

static int pps_servo_enable(int fd, int enable);

void usage(void) {
	printf("usage: tst-servo [-d [-p Kp] [-i Ki]] [-o [-p Kp] [-i Ki]]\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int ch, err, tcfd;
	int drift = 0, offset = 0;
	int gotdKp = 0, gotdKi = 0, gotoKp = 0, gotoKi = 0;
	uint32_t dKp, dKi, oKp, oKi;
	struct timecard_control gc;

	while((ch = getopt(argc, argv, "di:op:")) != -1)
		switch(ch) {
		case 'd':
			drift = 1;
			offset = 0;
			break;
		case 'o':
			offset = 1;
			drift = 0;
			break;
		case 'i':
			if (drift) {
				gotdKi = 1;
				dKi = strtoul(optarg, NULL, 0);
			} else if (offset) {
				gotoKi = 1;
				oKi = strtoul(optarg, NULL, 0);
			} else {
				printf("drift (-d) or offset (-o) must be specified first!\n");
				usage();
			}
			break;
		case 'p':
			if (drift) {
				gotdKp = 1;
				dKp = strtoul(optarg, NULL, 0);
			} else if (offset) {
				gotoKp = 1;
				oKp = strtoul(optarg, NULL, 0);
			} else {
				printf("drift (-d) or offset (-o) must be specified first!\n");
				usage();
			}
			break;
		case '?':
		default:
			usage();
		}
	if(optind < argc)
		usage();

	tcfd = open("/dev/timecard0", O_RDWR);
	if (tcfd == -1) {
		perror("open /dev/timecard0 failed");
		exit (EIO);
	}

	memset(&gc, 0, sizeof(struct timecard_control));
	gc.read = TC_CLK_SERVO_ADJ;
	err = ioctl(tcfd, TCIOCCONTROL, (caddr_t)&gc);
	if (err) {
		perror("ioctl TCIOCCONTROL\n");
		exit (EIO);
	}

	printf("before: offset Kp 0x%X Ki 0x%X drift Kp 0x%X Ki 0x%X\n",
	    gc.clk_servo_offset_Kp,
	    gc.clk_servo_offset_Ki,
	    gc.clk_servo_drift_Kp,
	    gc.clk_servo_drift_Ki);

	if (gotdKp || gotdKi || gotoKp || gotoKi) {
		printf("changing:");

		if (gotoKp || gotoKi)
			printf(" offset");
		if (gotoKp) {
			printf(" Kp 0x%X", oKp);
	    		gc.clk_servo_offset_Kp = oKp;
		}
		if (gotoKi) {
			printf(" Ki 0x%X", oKi);
	    		gc.clk_servo_offset_Ki = oKi;
		}
		if (gotdKp || gotdKi)
			printf(" drift");
		if (gotdKp) {
			printf(" Kp 0x%X", dKp);
	    		gc.clk_servo_drift_Kp = dKp;
		}
		if (gotdKi) {
			printf(" Ki 0x%X", dKi);
	    		gc.clk_servo_drift_Ki = dKi;
		}
		printf("\n");
		pps_servo_enable(tcfd, 0);
		gc.write = TC_CLK_SERVO_ADJ;
		err = ioctl(tcfd, TCIOCCONTROL, (caddr_t)&gc);
		if (err) {
			perror("ioctl TCIOCCONTROL\n");
			exit (EIO);
		}
		printf("after: offset Kp 0x%X Ki 0x%X drift Kp 0x%X Ki 0x%X\n",
		    gc.clk_servo_offset_Kp,
		    gc.clk_servo_offset_Ki,
		    gc.clk_servo_drift_Kp,
		    gc.clk_servo_drift_Ki);
		pps_servo_enable(tcfd, 1);
	}

	close(tcfd);
	return 0;
}

static int pps_servo_enable(int fd, int enable)
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
	err = ioctl(fd, TCIOCCONTROL, &tc);

	return err;
}
