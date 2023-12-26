#!/bin/sh

# PROVIDE: timecard
# REQUIRE: DAEMON
# BEFORE: ntpd
# KEYWORD:

#
# Add the following lines to /etc/rc.conf to enable timecard:
#
# timecard_enable (bool):	Set to "NO" by default.
#				Set it to "YES" to enable timecard.
# timecard_flags (str):	Set to "" by default.
#				See timecard(8) for details.
# timecard_logfile (str):
# timecard_driftfile (str):

. /etc/rc.subr

name="timecard"
desc="Synchronize kernel time to the TimeCard and provide shm(28) driver for ntpd"
rcvar="timecard_enable"
required_modules="axi_iic:timecard/axi_iic iic:iicbus/iic axi_spi_timecard:timecard/spi uart_timecard:timecard/uart timecard:pci/timecard"
# ? spigen.ko needs hints

pidfile="/var/run/${name}.pid"
procname="/usr/local/sbin/${name}"
command=/usr/sbin/daemon
command_args="-f -c -T timecard -p ${pidfile} ${procname}"

start_cmd="timecard_start"

timecard_wait()
{
	local to=30
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

timecard_start()
{
	if [ -n "${timecard_driftfile}" ]; then
		#command_args="${command_args} -d ${timecard_driftfile}"
		timecard_flags="${timecard_flags} -d ${timecard_driftfile}"
	fi
	if [ -n "${timecard_logfile}" ]; then
		timecard_flags="${timecard_flags} -l ${timecard_logfile}"
	fi
	#echo "flags $command_args $timecard_flags"
	timecard_wait
	# XXX Is the best place? More checks (GNSS synced, etc. ) before done?
	/sbin/sysctl kern.timecounter.hardware="TimeCard"
	$command $command_args $timecard_flags
}

load_rc_config $name
run_rc_command "$1"

