#include "KeyboardTyper.h"

void KeyboardTyper::init() {
    while(true) {
        std::vector<Vector3> points; //three points on the keyboard

        while(points.size() != 3) {
            auto res = randomPress();
            if(res.type != Key::NotOnKeyboard) {
                points.push_back(res.pos);
            }
        }
        //TODO: magic constant to adjust
        if(Plane::getOrtho(points[0], points[1], points[2]).norm() > 1e-5) {
            _plane = Plane(points[0], points[1], points[2]);
            break;
        }
    }
    while(hasTwoNonEmpty(0) || hasTwoNonEmpty(1) || hasTwoNonEmpty(2));
}

void KeyboardTyper::handleKeypress(KeyboardTyper::Key key) {
    if(key.type == Key::GoodKey) {
        _points[key.row][key.col].push_back(_plane.proj2d(key.pos));
    }
}

static Vector3 barycenter(const std::vector<Vector3>& pts) {
    assert(!pts.empty());
    Vector3 res{0, 0, 0};
    for(const Vector3& v : pts) {
        res += v;
    }
    return res/pts.size();
}

std::optional<KeyboardTyper::Linspace> KeyboardTyper::optimize(int line, double alpha) {
    int firstNotEmpty = -1, lastNotEmpty = -1;
    size_t n_points = 0;

    for(size_t i = 0; i < _points[line].size(); ++i) {
        n_points += _points[line][i].size();

        if(!_points[line][i].empty()) {
            if(firstNotEmpty == -1) {
                firstNotEmpty = i;
            }
            lastNotEmpty = i;
        }

        if(firstNotEmpty == lastNotEmpty) {
            return std::nullopt;
        }
    }


    Linspace res;

    Vector3 firstBary = barycenter(_points[line][firstNotEmpty]);
    Vector3 lastBary = barycenter(_points[line][lastNotEmpty]);
    res.dir = (lastBary - firstBary) / (lastNotEmpty - firstNotEmpty);
    res.from = firstBary - res.dir*firstNotEmpty;

    //TODO: magic number of steps to remove?
    for(size_t step = 0; step < 1000; ++step) {
        Vector3 dFrom{0, 0, 0}, dDir{0, 0, 0};
        for(size_t i = 0; i < _points[line].size(); ++i) {
            for(const Vector3& pt : _points[line][i]) {
                dFrom += -(pt-res.from-i*res.dir);
                dDir += -i*(pt-res.from-i*res.dir);
            }
        }

        res.from -= alpha*dFrom/n_points;
        res.dir -= alpha*dDir/n_points;
    }

    return res;
}

KeyboardTyper::Key KeyboardTyper::randomPress() const {
    //TODO
    //return followTraj(Trajectory::line(Vector3{0,0,0}, Vector3{0,0,0}));
}

KeyboardTyper::Key KeyboardTyper::followTraj(Trajectory traj) const {
    //TODO
}

bool KeyboardTyper::hasTwoNonEmpty(int line) {
    size_t nbNonEmpty = 0;
    for(size_t i = 0; i < _points[line].size(); ++i) {
        nbNonEmpty += !_points[line][i].empty();
    }
    return nbNonEmpty >= 2;
}
