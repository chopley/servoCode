# -*- coding: utf-8 -*-
#pythoncrosscan.py
#rough script based cross scan for the c-bass antenna
import sys
import os
import time
print sys.argv
az = float(sys.argv[1])
alt = float(sys.argv[2])

print az, alt
timescan = 300
timescana = timescan
time2 = 40
time2a = time2
widthaz = 0.1
widthalt=0.1	
az_res = 0.05
alt_res = 0.05
min_az = az-widthaz
max_az = az+widthaz
min_alt=alt-widthalt
max_alt=alt+widthalt
az_start = min_az
alt_start = min_alt
#output = "hello",`az`
#print output


os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(min_az,min_az,alt_start,alt_start,timescan))
time.sleep(40)
while(alt_start<=max_alt):
  os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(min_az,max_az,alt_start,alt_start,timescan))
  time.sleep(timescana)
  os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(max_az,max_az,alt_start,alt_start+alt_res,time2))
  time.sleep(time2a)
  alt_start = alt_start+alt_res
  os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(max_az,min_az,alt_start,alt_start,timescan))
  time.sleep(timescan)
  os.system('./give_command_tcp c-bass2 117 %f %f %f %f %f'%(min_az,min_az,alt_start,alt_start+alt_res,time2))
  alt_start = alt_start+alt_res
  time.sleep(time2a)
  