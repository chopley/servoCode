Servo Code for the Southern Antenna
=========

Userspace
=========
make development in the home directory

Kernel Module
=========
There is a copy of the Kubuntu 9.04 DVD install iso located in ISO- This can
be installed

The following directories include files to do a reinstall of the CBASS
toolchain

1. Python
> sudo apt-get install python
> sudo apt-get install python-matplotlib
> sudo apt-get install python-scipy
> sudo apt-get install python-numpy
> sudo apt-get install python-tk python-dev

2. Stellarium
sudo apt-get install stellarium

3. Cross Compilers
See folder attached-Requires additional C libraries to be apt-getted-these
are described in CROSS_COMPILERS/README

4. Slalib
sudo apt-get install sudo apt-get install python-tk python-numpy python-dev

a. Compile for ARM (see slalib_c/README_by_CHARLES/README for instructions)
b. Compile for PC (as above)
c. Compile for python (see pyslalib-1.0/README.txt)

5. SAMBA (AT91) [NOTE these are from my written notes and may not be
perfect]
a. Attached ISO has all the programs required
b. Install AT91-ISP (ATMEL/Install AT91_ISP.exe)
c. Install USB driver (ATMEL/samba-driver/BasicUSB-6124
d. Let windows automatically install the USB after this
e. Run Samba v2.5 and connect as appropriate 
f. Copy Sam9_l9260_samba from CD to harddisk
g. Copy uImage2.6.26 to uImage_write in SAM9_l9260_samba- This can be found
in the linux_sources/linux-2.6.26_CBASS/arch/arm/boot/uImage after kernel is
properly compiled or a pre-compiled version is located in the parent
directory
h. Copy NANDFLASH.tcl from ATMEL/ on CD to lib/AT91SAM9260EK/ on PC Samba
installation
i. Remove NANDF_E and DF_E jumpers
j. Power on Board
k Replace Jumpers- any tcl problems mean jumpers have not been replaced or
that NANDFLASH.tcl has not been updated in the lib/AT91SAM9260EK
l Run AT91SAM9260...linux...

6. ssh
sudo apt-get install ssh

7. wvdial
sudo apt-get install wvdial

8. ntp
sudo apt-get install ntp
check /etc/resolv.conf 
change /etc/ntp.conf with 
server 127.127.1.1
fudge 127.127.1.1 stratum 9

9. ipkungfu

10. squid

11. dnsmasq ipmasq

TESTING SYSTEM
1. Try compiling the ARM kernel in linux_sources/linux-2.6.26_CBASS- use
#> make ARCH=arm CROSS_COMPILE=arm-linux- INSTALL_MOD_PATH=/?PATH?/ modules

2. Try running the python control gui (embed/cbass_control_gui.py)

3. Try the cross compilers (embed/controller_progs/
#> make

If all the above tests work then the bulk of the job is done!
