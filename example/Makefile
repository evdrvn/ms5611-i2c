all: ms5611-i2c-example 

ms5611-i2c-example: ../src/ms5611-i2c.c src/main.c
	gcc -Wall -O2 -I../src -Iext/evdsptc/src -Lext/evdsptc/build ../src/ms5611-i2c.c src/main.c -static -levdsptc -lm -lpthread -o ms5611-i2c-example 

clean:
	rm -f ms5611-i2c-example
