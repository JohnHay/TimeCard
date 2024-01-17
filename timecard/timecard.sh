#!/bin/sh

# PROVIDE: timecard
# REQUIRE: syslogd
# BEFORE: SERVERS ntpd
# KEYWORD:

#
# Add the following lines to /etc/rc.conf to enable timecard:
#
# timecard_enable (bool):	Set to "NO" by default.
#				Set it to "YES" to enable timecard.
# timecard_flags (str):	Set to "" by default.
#				See timecard(8) options for details.
# timecard_logfile (str):
# timecard_driftfile (str):
# timecard_hints (str):		See timecard_default_hints below.
# timecard_modules (str):	See timecard_default_modules below.
# timecard_pps_symlink (str):	Source_dev target_dev, eg.
#				"/dev/timecard0 /dev/pps0"
# timecard_servo (str):		See options for tc-adj-servo(8).
# timecard_sync_wait (num):	Seconds to wait for card to sync.
#				Default is 30 seconds.

. /etc/rc.subr

name="timecard"
desc="Synchronize kernel time to the TimeCard and provide shm(28) driver for ntpd"
rcvar="timecard_enable"

timecard_default_modules="
	axi_iic:timecard/axi_iic
	iic:iicbus/iic
	axi_spi_timecard:timecard/spi
	spigen:spibus/spigen
	uart_timecard:timecard/uart
	timecard:pci/timecard
"
timecard_default_hints="
	hint.spigen.0.at=spibus0
	hint.spigen.0.clock=1000000
	hint.spigen.0.cs=0
	hint.spigen.0.mode=0
"

pidfile="/var/run/${name}.pid"
procname="/usr/local/sbin/${name}"
command=/usr/sbin/daemon
command_args="-f -c -T timecard -p ${pidfile} ${procname}"
tc_adj_servo="/usr/local/sbin/tc-adj-servo"

start_cmd="timecard_start"
start_precmd="timecard_prestart"

load_rc_config $name

# Set defaults
: ${timecard_enable:="NO"}
: ${timecard_hints:=${timecard_default_hints}}
: ${timecard_sync_wait:=30}
: ${required_modules:=${timecard_modules}}
: ${required_modules:=${timecard_default_modules}}

timecard_wait()
{
	local to=${timecard_sync_wait}
	local insync
	local utcvalid
	local fixok

	while [ ${to} -gt 0 ]; do
		insync=`sysctl -n dev.timecard.0.status.clk_in_sync`
		utcvalid=`sysctl -n dev.timecard.0.status.tod_utc_valid`
		fixok=`sysctl -n dev.timecard.0.status.gnss_fix_ok`
		if [ ${insync} -eq 1 -a ${utcvalid} -eq 1 -a ${fixok} -eq 1 ]; then
			break;
		fi
		sleep 1
		to=$((${to}-1))
	done
}

timecard_prestart()
{
	local _h
	for _h in ${timecard_hints}; do
		kenv -q ${_h}
	done
}

timecard_start()
{
	if [ ! -c "/dev/timecard0" ]; then
		echo "timecard driver did not load!"
		return 6
	fi
	if [ -n "${timecard_driftfile}" ]; then
		#command_args="${command_args} -d ${timecard_driftfile}"
		timecard_flags="${timecard_flags} -d ${timecard_driftfile}"
	fi
	if [ -n "${timecard_logfile}" ]; then
		timecard_flags="${timecard_flags} -l ${timecard_logfile}"
	fi
	if [ -n "${timecard_servo}" ]; then
		${tc_adj_servo} ${timecard_servo}
	fi
	if [ -n "${timecard_pps_symlink}" ]; then
		ln -f -s ${timecard_pps_symlink}
	fi
	timecard_wait
	/sbin/sysctl kern.timecounter.hardware="TimeCard"
	$command $command_args $timecard_flags
}

run_rc_command "$1"

