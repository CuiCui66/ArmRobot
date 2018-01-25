
#pragma once

class Motor {
    char* m_path;
    long def_pos;

    public:
    Motor() = delete;
    Motor(const char* name);

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

