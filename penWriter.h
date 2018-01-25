#pragma once

#include <map>
#include <vector>
#include "geometry.h"

const Vector3 PAPER_START = {0, 300, 0}; // TODO adjust, in mm
const Vector3 WRITING_DIR = {1, -1, 0};
const Vector3 RISE = {0, 0, 100};

const double ROT = sqrt(2) / 2;

class PenWriter {
    private:
        double size;
        std::map<char, std::vector<Trajectory> > font;
        Trajectory getTraj(char, Vector3);
    public:
        void loadFont(std::string);
        void setSize(double); // in mm
        void write();
        PenWriter(double, std::string);
};
