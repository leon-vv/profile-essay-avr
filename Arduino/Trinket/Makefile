
default: upload
	
main.o: main.c
	avr-gcc -Wall -Og -DF_CPU=16000000 -mmcu=attiny85 $^ -o build/$@

simulate: main.o
	~/Development/C/simavr/simavr/run_avr -mcu attiny85 -f 16000000 -t ./build/main.o
	gtkwave gtkwave_trace.vcd

main-asm: main.c
	avr-gcc -Wall -Og -S -DF_CPU=16000000 -mmcu=attiny85 $^ -o build/$@.s

upload: main
	avr-objcopy -O ihex -R .eeprom build/$^ build/$^.hex
	/home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude -p attiny85 -c usbtiny -C /home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude.conf -U flash:w:build/$^.hex
