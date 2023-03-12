#!/bin/sh

PORT=$1
TERMIOS=b115200,echo=0,icrnl=0,icanon=0,isig=0,onlcr=0 #,opost=0,ixon=0,iexten=0,echoe=0,echok=0,min=0,echoctl=0,echoke=0

touch /tmp/done_uploading

while rlwrap -a -m socat FILE:$PORT,$TERMIOS STDOUT; [ $? -ne 130 ]; do
	echo waiting for upload finish
	inotifywait -qq /tmp/done_uploading
done
