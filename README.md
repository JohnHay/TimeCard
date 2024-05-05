### Summary
This is a FreeBSD driver for the Open Compute Project (OCP) Time Appliance Project (TAP)
[Time Card](https://github.com/opencomputeproject/Time-Appliance-Project/tree/master/Time-Card)
 
### The software consists of two main parts:
- The kernel module timecard(4):
  - Provides access to the card’s functionality,
  - Implements a timecounter, “TimeCard”, using the timecounters(4) API,
  - Make the PPS signal and timestamp available with the RFC 2783 Pulse Per Second (PPS) API,
  - Provides a timecard bus to which uart(4), iicbus(4) and spibus(4) drivers can attach.

- A daemon timecard(8):
  - Provides a shm(28) driver for ntpd(8),
  - Synchronize the kernel time to the timecard(4) time,
  - Discipline or train the clock on board of the TimeCard through the iic(4) Clock interface.

### Building from source:
To build requires a FreeBSD 14.0 or later, environment with the FreeBSD source in /usr/src.
Extract (git pull, clone, tar…) this source and change into that directory.
```
make # to compile everything as any user
make package # to build a FreeBSD package of it all
```

The package includes rc.d startup files that can be steered from /etc/rc.conf. For example:
```
timecard_enable="YES"
timecard_logfile="/var/log/tc/tc.log"
timecard_driftfile="/var/log/tc/driftfile.txt"
timecard_flags="-c -h -s -t -v"
```
To configure ntpd(8) to use the timecard through the shm interface, add the following lines to /etc/ntp.conf:
```
server 127.127.28.0 minpoll 4 maxpoll 4 prefer
fudge 127.127.28.0 refid TC flag4 1
```
