#!/bin/sh

PORT=$1
TERMIOS=b115200,echo=0,icrnl=0

touch /tmp/done_uploading

while rlwrap socat FILE:$PORT,$TERMIOS STDOUT; [ $? -ne 130 ]; do
	echo waiting for upload finish
	inotifywait -qq /tmp/done_uploading
done
