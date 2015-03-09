CC = gcc
CFLAGS = -pedantic -Wall -std=c99 -O3
INCLUDE = -I$(shell pwd)/../manual -I$(shell pwd)/../robot
OBJECTS = core.o
TARGET = core

all: subdirs

subdirs:
	make -C $(shell pwd)/../manual
	make -C $(shell pwd)/../robot


