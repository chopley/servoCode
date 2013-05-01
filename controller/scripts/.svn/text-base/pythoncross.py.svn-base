# -*- coding: utf-8 -*-
#pythoncrosscan.py
#rough script based cross scan for the c-bass antenna
import sys
import os
import time
#print sys.argv
az = float(sys.argv[1])
alt = float(sys.argv[2])
print az, alt
timescan = 30
timescana = timescan+5
time2 = 10
time2a = time2+5
widthaz = 1
widthalt=1
az_res = 0.1
alt_res = 0.1
min_az = az-widthaz
max_az = az+widthaz
min_alt=alt-widthalt
max_alt=alt+widthalt
az_start = min_az
alt_start = min_alt
#output = "hello",`az`
#print output

os.system('./give_command_tcp c-bass2 117 %f %f %f %f %i'%(min_az,max_az,alt_start,alt_start,timescan)

#while(alt_start<max_alt):
 #os.system('./give_command_tcp c-bass2 117 %f %f %f %f %i'%(min_az,max_az,alt_start,alt_start,timescan)
 #time.sleep(30)
 #os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(max_az,max_az,alt_start,alt_start+alt_res,time2)
 #alt_start = alt_start+alt_res
 #time.sleep(time2a)
 #os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(max_az,min_az,alt_start,alt_start,timescan)
 #time.sleep(timescana)
 