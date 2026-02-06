CC = gcc
CFLAGS = -Wall -Wextra -std=c11

SRC = src/main.c
OUT = build/main.exe

all:
	if not exist build mkdir build
	$(CC) $(CFLAGS) $(SRC) -o $(OUT)

clean:
	rmdir /S /Q build
