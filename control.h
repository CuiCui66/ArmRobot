
#pragma once
#include "geometry.h"

class Motor {
    char* m_path;
    long def_pos;

    public:
    Motor() = delete;
    Motor(const char* name);
    ~Motor();

    // Basic control
    void speed(long sp);
    void duty(long dt);
    void reset();
    void stop();
    void set(long st);

    // Information
    bool stalled() const;
    bool holding() const;
    long speed() const;
    long position() const;

    // Advanced control
    void init_pos();
    void reset_pos();
};

// Handle motors globally, and do the conversion betweens the unities of the
// motors and those of the geometry module
class Robot {
    Configuration m_angles;

    public:
    Motor base, shoulder, elbow, wrist;

    Robot();

    // Information
    Configuration configuration();
    Configuration speed();

    // Control
    void init();
    void stop();
    void applyConfigurationSpeed(const Configuration& conf);
    void point(const Configuration& conf);
};

