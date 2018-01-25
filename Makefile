all: test

test: main.cpp geometry.cpp geometry.h control.cpp control.h KeyboardTyper.cpp KeyboardTyper.h
	clang++ -std=c++17 -g -Wall -Wextra main.cpp geometry.cpp control.cpp KeyboardTyper.cpp -o test
