
#include "control.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cassert>
extern "C" {
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
}

using namespace std;
#define WRSTR(fd, str) { int n = write((fd), (str), strlen(str)); assert(n != -1); }

//  __  __       _             
// |  \/  | ___ | |_ ___  _ __ 
// | |\/| |/ _ \| __/ _ \| '__|
// | |  | | (_) | || (_) | |   
// |_|  |_|\___/ \__\___/|_|   
// 
                            
Motor::Motor(const char* name)
    : m_path(NULL), def_pos(0)
{
    char buffer[256];
    int fd;

    strcpy(buffer, name);
    strcat(buffer, "/stop_action");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "hold");
    close(fd);

    stop();
    m_path = strdup(name);
}

void Motor::speed(long sp) {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/speed_sp");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    sprintf(buffer, "%li", sp);
    n = write(fd, buffer, strlen(buffer));
    assert(n != -1);
    close(fd);

    strcpy(buffer, m_path);
    strcat(buffer, "/command");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "run-forever");
    close(fd);
}

void Motor::duty(long dt) {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/duty_cycle_sp");
    fd = open(buffer, O_RDONLY);
    assert(fd != -1);
    sprintf(buffer, "%li", dt);
    n = write(fd, buffer, strlen(buffer));
    assert(n != -1);
    close(fd);
}

void Motor::reset() {
    char buffer[256];
    int fd;

    strcpy(buffer, m_path);
    strcat(buffer, "/command");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "reset");
    close(fd);
}

void Motor::stop() {
    char buffer[256];
    int fd;

    strcpy(buffer, m_path);
    strcat(buffer, "/command");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "stop");
    close(fd);
}

bool Motor::stalled() {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/state");
    fd = open(buffer, O_RDONLY);
    assert(fd != -1);
    n = read(fd, buffer, 256);
    assert(n != -1);
    buffer[n] = '\0';
    close(fd);

    return strstr(buffer, "stalled") != NULL;
}

long Motor::speed() {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/speed");
    fd = open(buffer, O_RDONLY);
    n = read(fd, buffer, 256);
    close(fd);
    assert(n != -1);
    buffer[n] = '\0';
    return strtol(buffer, NULL, 10);
}

long Motor::position() {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/position");
    fd = open(buffer, O_RDONLY);
    assert(fd != -1);
    n = read(fd, buffer, 256);
    assert(n != -1);
    buffer[n] = '\0';
    close(fd);

    return strtol(buffer, NULL, 10) - def_pos;
}

void Motor::init_pos() {
    duty(40);
    while(!stalled());

    def_pos = 0;
    def_pos = position();
}


