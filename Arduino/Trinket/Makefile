
default: upload
	
main: main.c
	avr-gcc -Wall -O2 -DF_CPU=8000000 -mmcu=attiny85 $^ -o build/$@

main-asm: main.c
	avr-gcc -Wall -Og -S -DF_CPU=8000000 -mmcu=attiny85 $^ -o build/$@.s

upload: main
	avr-objcopy -O ihex -R .eeprom build/$^ build/$^.hex
	/home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude -p attiny85 -c usbtiny -C /home/leonvv/Development/Arduino/Trinket_Arduino_Linux/trinket-arduino-1.0.5/hardware/tools/avrdude.conf -U flash:w:build/$^.hex
