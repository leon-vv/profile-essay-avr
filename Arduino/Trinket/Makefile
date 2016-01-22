
default: main.o
	
main.o: main.c
	avr-gcc -Wall -O2 -DF_CPU=16000000 -mmcu=attiny85 -g $^ -o build/$@

main-asm: main.c
	avr-gcc -Wall -O2 -S -DF_CPU=16000000 -mmcu=attiny85 $^ -o build/$@.s

upload: main.o
	avr-objcopy -O ihex -R .eeprom build/$^ build/$^.hex
	/home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude -p attiny85 -c usbtiny -C /home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude.conf -U flash:w:build/$^.hex
