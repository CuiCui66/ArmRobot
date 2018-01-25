
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

    // Information
    bool stalled();
    long speed();
    long position();

    // Advanced control
    void init_pos();
};

// Handle motors globally, and do the conversion betweens the unities of the
// motors and those of the geometry module
class Robot {
    Configuration m_def;

    public:
    Motor base, shoulder, elbow, wrist;

    Robot();

    // Information
    Configuration configuration();
    Configuration speed();
    Vector3 tipPositionCart();
    Vector3 tipSpeedCart();

    // Control
    void init();
    void applyConfigurationSpeed(const Configuration& conf);
};

