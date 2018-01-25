#pragma once

#include <optional>
#include "geometry.h"

class KeyboardTyper {
    public:
        struct Linspace {
            Vector3 from, dir;
        };

        struct Key {
            int row, col;
            Vector3 pos;
            enum {
                NotOnKeyboard, //self-descriptive
                BadKey, //on keyboard but non-letter key
                GoodKey, //on letter key
            } type;
        };

        void init();
        void handleKeypress(Key key);
        std::optional<Linspace> optimize(int line, double alpha = 0.001);
        Key randomPress() const;
        Key followTraj(Trajectory traj) const;

    private:
        bool hasTwoNonEmpty(int line);
        //The plane of the keyboard
        Plane _plane;
        //_points[i][j] = vector of points corresponding to the ith row and jth column key
        //It should be Vector2 but we use Vector3 with z=0
        std::vector<std::vector<std::vector<Vector3>>> _points;
};
