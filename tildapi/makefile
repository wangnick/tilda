# Tilda - A two-wheel balancing robot using Lego, Attiny, Raspberry Pi
# Copyright (C) 2013 Sebastian Wangnick
# See http://s.wangnick.de/doku.php?id=tilda for design details.

CC=gcc -Wall
CFLAGS=-c -g
LDFLAGS=-g
SOURCES=tilda.cpp mpulib.cpp drivelib.cpp euler.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=tilda
LIBS=-lm

all: $(SOURCES) $(EXECUTABLE) drive mpu
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

drive: drive.o drivelib.o
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

mpu: mpu.o mpulib.o euler.o
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

depend: .depend

.depend: $(SOURCES) drive.cpp mpu.cpp
	rm -f ./.depend
	$(CC) $(CFLAGS) -MM $^ > ./.depend

include .depend
