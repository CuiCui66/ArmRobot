
#include "control.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <iostream>
extern "C" {
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
}

using namespace std;
#define WRSTR(fd, str) { int n = write((fd), (str), strlen(str)); assert(n != -1); }

// ┏┳┓┏━┓╺┳╸┏━┓┏━┓
// ┃┃┃┃ ┃ ┃ ┃ ┃┣┳┛
// ╹ ╹┗━┛ ╹ ┗━┛╹┗╸

Motor::Motor(const char* name)
    : m_path(NULL), def_pos(0)
{
    m_path = strdup(name);

    char buffer[256];
    int fd;

    strcpy(buffer, name);
    strcat(buffer, "/stop_action");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "hold");
    close(fd);

    stop();
}

Motor::~Motor() {
    free(m_path);
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
    
void Motor::set(long st) {
    cout << m_path << " : " << st << endl;

    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/speed_sp");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "45");
    close(fd);

    st += def_pos;
    strcpy(buffer, m_path);
    strcat(buffer, "/position_sp");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    sprintf(buffer, "%li", st);
    n = write(fd, buffer, strlen(buffer));
    assert(n != -1);
    close(fd);

    strcpy(buffer, m_path);
    strcat(buffer, "/command");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "run-to-abs-pos");
    close(fd);

    while(!holding());
}

void Motor::duty(long dt) {
    char buffer[256];
    int fd, n;

    strcpy(buffer, m_path);
    strcat(buffer, "/duty_cycle_sp");
    fd = open(buffer, O_WRONLY);
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

bool Motor::stalled() const {
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

bool Motor::holding() const {
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

    return strstr(buffer, "holding") != NULL;
}

long Motor::speed() const {
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

long Motor::position() const {
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
    char buffer[256];
    int fd;

    duty(0);
    strcpy(buffer, m_path);
    strcat(buffer, "/command");
    fd = open(buffer, O_WRONLY);
    assert(fd != -1);
    WRSTR(fd, "run-direct");
    close(fd);
    duty(30);
    while(!stalled());

    stop();
    reset_pos();
}

void Motor::reset_pos() {
    def_pos = 0;
    def_pos = position();
}


// ┏━┓┏━┓┏┓ ┏━┓╺┳╸
// ┣┳┛┃ ┃┣┻┓┃ ┃ ┃
// ╹┗╸┗━┛┗━┛┗━┛ ╹

#define DEG2RAD(x) (((double)(x)) * pi / 180.0)
#define RAD2DEG(x) (long int)((x) * 180.0 / pi)

Robot::Robot()
    : base(getenv("MCD")), shoulder(getenv("MCA")), elbow(getenv("MCB")), wrist(getenv("MCC"))
{
    m_angles.base     = 0;
    m_angles.shoulder = DEG2RAD(157) + 0.15;
    m_angles.elbow    = DEG2RAD(150) + 0.136;
    m_angles.wrist    = DEG2RAD(93 - 20);
}

Configuration Robot::configuration() {
    Configuration ret;
    ret.base     = DEG2RAD(base.position()  * 3 / 35);
    ret.shoulder = DEG2RAD(shoulder.position()  / 9);
    ret.elbow    = DEG2RAD(elbow.position()     / 3);
    ret.wrist    = DEG2RAD(wrist.position() * 9 / 25);
    return ret + m_angles;
}

Configuration Robot::speed() {
    Configuration ret;
    ret.base     = DEG2RAD(base.speed()  * 3 / 35);
    ret.shoulder = DEG2RAD(shoulder.speed()  / 9);
    ret.elbow    = DEG2RAD(elbow.speed()     / 3);
    ret.wrist    = DEG2RAD(wrist.speed() * 9 / 25);
    return ret;
}

void Robot::init() {
    base.stop();
    base.reset_pos();
    shoulder.init_pos();
    elbow.init_pos();
    wrist.init_pos();
}

void Robot::applyConfigurationSpeed(const Configuration& conf) {
    base    .speed(RAD2DEG(conf.base)     * 35 / 3);
    shoulder.speed(RAD2DEG(conf.shoulder) * 9);
    elbow   .speed(RAD2DEG(conf.elbow)    * 3);
    wrist   .speed(RAD2DEG(conf.wrist)    * 25 / 9);
}

void Robot::point(const Configuration& conf) {
    Configuration dest = conf - m_angles;
    base    .set( RAD2DEG(dest.base)     * 35 / 3);
    shoulder.set( RAD2DEG(dest.shoulder) * 9);
    elbow   .set( RAD2DEG(dest.elbow)    * 3);
    wrist   .set(-RAD2DEG(dest.wrist)    * 25 / 9);
}

