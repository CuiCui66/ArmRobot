OBJS=geometry.o control.o main.o penWriter.o
HEADERS=geometry.h control.h fraction.h penWriter.h
CC=arm-linux-gnueabi-g++
CFLAGS=-std=c++14 -g -Wall -Wextra

all: test

test: $(OBJS) $(HEADERS)
	$(CC) $(CFLAGS) $^ -o $@

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) $< -c -o $@

clean:
	@touch $(OBJS)
	rm $(OBJS)

