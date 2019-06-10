LDFLAGS := -lwiringPi

all:
	gcc main.c $(LDFLAGS)
