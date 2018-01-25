all: test

test: main.cpp geometry.cpp geometry.h control.cpp control.h
	clang++ -std=c++17 -g -Wall -Wextra main.cpp geometry.cpp control.cpp -o test
