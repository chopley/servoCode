#!/bin/bash
#sh pid_vals.sh
while true;
do
	./give_command_tcp c-bass2 112 $1 $2 $3 $4 $5 $6
	sleep 2
done