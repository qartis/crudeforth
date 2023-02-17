PORT=/dev/ttyUSB0
BOARD=esp32:esp32:esp32
SHELL=/bin/bash -o pipefail

all: build/crudeforth.ino.bin

build/crudeforth.ino.bin: crudeforth.ino
	arduino-cli compile -b $(BOARD) --build-path ./build/ -e -v | ./tools/truncatelines
	@rm -f debug.cfg debug_custom.json esp32.svd

.PHONY: load
load: build/crudeforth.ino.bin
	killall -q rlwrap || true
	arduino-cli upload -b $(BOARD) --input-dir ./build/ -p $(PORT)
	touch /tmp/done_uploading

.PHONY: clean
clean:
	rm -R ./build/

.PHONY: monitor
monitor:
	@./tools/serial-monitor.sh $(PORT)
