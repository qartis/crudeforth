PORT=/dev/ttyUSB0
BOARD=esp32:esp32:esp32

all: build/crudeforth.ino.bin

build/crudeforth.ino.bin: crudeforth.ino
	arduino-cli compile -b $(BOARD) --build-path ./build/ -e -v | tools/truncatelines
	@rm -f debug.cfg debug_custom.json esp32.svd

.PHONY: load
load: build/crudeforth.ino.bin
	arduino-cli upload -b $(BOARD) --input-dir ./build/ -p $(PORT)

.PHONY: clean
clean:
	rm -R ./build/

.PHONY: monitor
monitor:
	rlwrap socat FILE:$(PORT),b115200 STDOUT
