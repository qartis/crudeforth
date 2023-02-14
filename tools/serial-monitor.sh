#!/bin/sh

touch /tmp/done_uploading
PORT=$1

while rlwrap socat FILE:$PORT,b115200 STDOUT; [ $? -ne 130 ]; do
	echo waiting for upload finish
	inotifywait -qq /tmp/done_uploading
done
