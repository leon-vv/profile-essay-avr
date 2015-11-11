
default: upload
	sh ./serial.sh /dev/ttyUSB0 9600
	
main: main.c
	mkdir -p build
	avr-gcc -Wall -Os -DF_CPU=16000000 -mmcu=atmega328p $^ -o build/$@

upload: main
	avr-objcopy -O ihex -R .eeprom build/$^ build/$^.hex
	avrdude -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 57600 -U flash:w:build/$^.hex
