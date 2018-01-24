all: test

test: main.cpp geometry.cpp geometry.h
	clang++ -std=c++17 -g -Wall -Wextra main.cpp geometry.cpp -o test
