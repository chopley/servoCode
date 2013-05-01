#!/bin/bash
hhh = "hello" 
i=0
while [ $(($1+$i)) -lt $(($1+1000)) ];
do
	echo ./give_command_tcp c-bass2 102 $(($1+$i)) $3
	sleep 1
	i=$(($i+100))
done


