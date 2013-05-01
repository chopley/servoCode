#!/usr/bin/python
# -*- coding: utf-8 -*-
#  chmod a+x helloworld.py

import socket, threading, thread

#import slalib as s
import time as t
import string
import scipy as scipy
import pylab as plab 
import cmath
import math
from ctypes import *
from datetime import datetime


#import Image

#import example

#define constants here




global app
global current_time
global second_counts
global command_ack_received
global datafile
global gtime
global az_encoder
global plotupdate
global alt_encoder

HOST = ''      # Symbolic name meaning the local host
PORT = 1500          # Arbitrary non-privileged port


def getipaddrs(hostname):
    result = socket.getaddrinfo(hostname, None, 0, socket.SOCK_STREAM)
    return [x[4][0] for x in result]

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

hostname = socket.gethostname()
try:
    print "IP addresses:", ", ".join(getipaddrs(hostname))
except socket.gaierror, e:
    print "Couldn't not get IP addresses:", e
    
print HOST
s.bind((HOST, PORT))
s.listen(1)
datafile = datetime.today()
data =str(datafile)
data += '.txt'
data = data.rstrip(" ")
print "Saving data to ",data
dout = 'log_control/'
dout +=data
f = open(dout,'w')


#create an instance of the class used to parse incoming data from the controller device

#sock.setblocking(1);
while 1:
    conn, addr = s.accept()
    time1 = 556.
    tstringold = 55.
    tstring =55.
    #data = conn.recv(3000)
    while 1:
      alist1 = []
      data = []
      data = conn.recv(4000)
   
      if not data : break
     # print data
      #tmp = mystring[1:-1]
      
      
      alist=data.rstrip("\x00\n")
     # print alist
      alist1=alist.rsplit(',')
      if len(alist1)>50:
	#print alist1[0],alist1[1],alist1[4],alist1[5],alist1[10],alist1[11],alist1[12],alist1[13],alist1[14],alist1[15],alist1[16],alist1[17],alist1[25],alist1[26]
	
	
	told = time1
	time1 = float(alist1[25])
	time2 = float(alist1[26])
	
	time1 = time1 + time2/1000000.
	tstringold = tstring
	try:
	  tstring = datetime.fromtimestamp(time1)
	except ValueError:
	  print "That was not a number."
	  tstring = "Fail time stamp generation"
	print >>f, tstring, alist1[0],alist1[1],alist1[2],alist1[3],alist1[4],alist1[5],alist1[6],alist1[7],alist1[10],alist1[11],alist1[12],alist1[13],alist1[14],alist1[15],alist1[16],alist1[17],alist1[25],alist1[26]

	print tstring, alist1[0],alist1[1],alist1[2],alist1[3],alist1[4],alist1[5],alist1[6],alist1[7],alist1[10],alist1[11],alist1[12],alist1[13],alist1[14],alist1[15],alist1[16],alist1[17],alist1[25],alist1[26],time1-told

    #cli_sock.send("Hello, person from %s" % cli_addr)




	
	