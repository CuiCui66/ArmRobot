#include "geometry.h"
#include <iostream>

using namespace std;

#define STUDS(x) ((double)(x) * 7.97)
double l1 = STUDS(25); // mm
double l2 = Vector2({STUDS(22),STUDS(3)}).norm(); // mm
double l3 = STUDS(9); // mm
double epsilon = 0.00001;
double pi = std::acos(-1);
Vector3 robotpos{0,0,0.0};
// O1 is the first horizontal axis.
// the must be O1pos minus the height of the hand.
Vector2 O1pos{STUDS(3),STUDS(30)};
double robotor = 0.0;


//  ____  ____
// |___ \|  _ \
//   __) | | | |
//  / __/| |_| |
// |_____|____/

Polar2 toPol(const Vector2& vec){
    return Polar2{vec.norm(),atan2(vec.y,vec.x)};
}
MovingPoint<Polar2> toPol(const MovingPoint<Vector2>& vec){
    Polar2 pos = toPol(vec.pos);
    return mp(pos,
              Polar2{vec.pos.dot(vec.spd)/vec.pos.norm(),
                      vec.pos.det(vec.spd)/(pos.r*pos.r)}
              );
}
Vector2 toVec(const Polar2& pol){
    assert(pol.r > 0);
    return Vector2{pol.r * cos(pol.theta), pol.r * sin(pol.theta)};
}
MovingPoint<Vector2> toVec(const MovingPoint<Polar2>& pol){
    Vector2 pos = toVec(pol.pos);
    Vector2 spd{pol.spd.r, pol.spd.theta * pol.pos.r};
    return mp(pos, spd.rotate(-pol.pos.theta));

}

//  _____ ____
// |___ /|  _ \
//   |_ \| | | |
//  ___) | |_| |
// |____/|____/

Vector3 toVec(const Cylindrical& cyl){
    Vector2 v = toVec(cyl.p);
    return Vector3{v.x,v.y,cyl.z};
}
MovingPoint<Vector3> toVec(const MovingPoint<Cylindrical>& cyl){
    auto v = toVec(mp(cyl.pos.p,cyl.spd.p));
    return mp(Vector3{v.pos.x,v.pos.y,cyl.pos.z},
              Vector3{v.spd.x,v.spd.y,cyl.spd.z});
};

Vector3 toVec(const Polar3& pol){
    return toVec(toCyl(pol));
}
MovingPoint<Vector3> toVec(const MovingPoint<Polar3>& pol){
    return toVec(toCyl(pol));
}

Polar3 toPol(const Vector3& vec){
    return toPol(toCyl(vec));
}
MovingPoint<Polar3> toPol(const MovingPoint<Vector3>& vec){
    return toPol(toCyl(vec));
}
Polar3 toPol(const Cylindrical& cyl){
    Polar2 pol = toPol(cyl.vertPlane());
    return Polar3{pol,cyl.p.theta};
}
MovingPoint<Polar3> toPol(const MovingPoint<Cylindrical>& cyl){
    auto pol = toPol(mp(cyl.pos.vertPlane(), cyl.spd.vertPlane()));
    return mp(Polar3{pol.pos, cyl.pos.p.theta},Polar3{pol.spd, cyl.spd.p.theta});
}

Cylindrical toCyl(const Vector3& vec){
    Polar2 pol = toPol(vec.lower());
    return Cylindrical{pol, vec.z};
}
MovingPoint<Cylindrical> toCyl(const MovingPoint<Vector3>& vec){
    auto pol = toPol(mp(vec.pos.lower(),vec.spd.lower()));
    return mp(Cylindrical{pol.pos,vec.pos.z}, Cylindrical{pol.spd,vec.spd.z});
}

Cylindrical toCyl(const Polar3& pol){
    Vector2 vp = toVec(pol.p);
    return Cylindrical{Polar2{vp.x,pol.theta},vp.y};
}
MovingPoint<Cylindrical> toCyl(const MovingPoint<Polar3>& pol){
    auto vp = toVec(mp(pol.pos.p,pol.spd.p));
    return mp(Cylindrical{Polar2{vp.pos.x,pol.pos.theta},vp.pos.y},
            Cylindrical{Polar2{vp.spd.x,pol.spd.theta},vp.spd.y});
}


//  _____                 _   _
// |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
// | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
// |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
// |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/


Vector3 toInternal(const Vector3& vec){
    return (vec - robotpos).rotatez(-robotor);
}

MovingPoint<Vector3> toInternal(const MovingPoint<Vector3>& vec){
    Vector3 pos = toInternal(vec.pos);
    Vector3 spd = vec.spd.rotatez(-robotor);
    return mp(pos,spd);
}


Vector3 fromInternal(const Vector3& vec){
    return (vec + robotpos).rotatez(robotor);
}

MovingPoint<Vector3> fromInternal(const MovingPoint<Vector3>& vec){
    Vector3 pos = fromInternal(vec.pos);
    Vector3 spd = vec.spd.rotatez(robotor);
    return mp(pos,spd);
}


double triangleAngle(double c1, double c2, double c3){
    double in = (c1*c1 + c2*c2 - c3*c3)/(2 * c1 * c2);
    assert(abs(in) <= 1);
    return acos(in);
}
MovingPoint<double> triangleAngle(MovingPoint<double> c1, double c2, double c3){
    auto in = (c1*c1 + c2*c2 - c3*c3)/(2 * c1 * c2);
    assert(abs(in.pos) <= 1);
    return acos(in);

}
MovingPoint<double> triangleAngle(double c1, double c2, MovingPoint<double> c3){
    auto in = (c1*c1 + c2*c2 - c3*c3)/(2 * c1 * c2);
    assert(abs(in.pos) <= 1);
    return acos(in);

}

Configuration toConfiguration(Cylindrical cyl){
    cyl -= O1pos;
    cyl += Vector2{0,l3};
    Polar3 pol = toPol(cyl);
    double shoulder = triangleAngle(pol.p.r, l1, l2) + pol.p.theta + pi/2;
    cout << " ta: " << triangleAngle(pol.p.r, l1, l2) << " " << pol.p.theta << endl;
    double elbow = triangleAngle(l1, l2, pol.p.r);
    return Configuration{pol.theta, shoulder, elbow, 2*pi-shoulder-elbow};
}


MovingPoint<Configuration> toConfiguration(MovingPoint<Cylindrical> cyl){
    cyl.pos -= O1pos;
    cyl.pos += Vector2{0,l3};
    auto pol = toPol(cyl);
    auto shoulder = triangleAngle(mp(pol.pos.p.r,pol.spd.p.r), l1, l2)
        + mp(pol.pos.p.theta, pol.spd.p.theta) + pi/2;
    auto elbow = triangleAngle(l1, l2, mp(pol.pos.p.r,pol.spd.p.r));
    return mp(Configuration{pol.pos.theta,shoulder.pos,elbow.pos, 2*pi-shoulder.pos - elbow.pos},
              Configuration{pol.spd.theta,shoulder.spd,elbow.spd, -shoulder.spd - elbow.spd}
              );
}
Cylindrical fromConfiguration(const Configuration& cfg){
    Cylindrical cyl;
    cyl.p.theta = cfg.base;
    Vector2 vp = O1pos
        + Vector2{0,l1}.rotate(pi - cfg.shoulder)
        + Vector2{0,l2}.rotate(2*pi - cfg.shoulder - cfg.elbow)
        + Vector2{0,l3}.rotate(3*pi - cfg.shoulder - cfg.elbow - cfg.wrist);
    cyl.p.r = vp.x;
    cyl.z = vp.y;
    return cyl;
}

MovingPoint<Cylindrical> fromConfiguration(const MovingPoint<Configuration>& cfg){
    MovingPoint<Cylindrical> cyl;
    cyl.pos = fromConfiguration(cfg.pos);
    cyl.spd.p.theta = cfg.spd.base;
    Vector2 vpd = Vector2{-l1,0}.rotate(pi- cfg.pos.shoulder) * (cfg.spd.shoulder)
        + Vector2{-l2,0}.rotate(2*pi - cfg.pos.shoulder - cfg.pos.elbow )
                         *(cfg.spd.shoulder + cfg.spd.elbow)
        + Vector2{-l3,0}.rotate(3*pi - cfg.pos.shoulder - cfg.pos.elbow - cfg.pos.wrist)
                         *(cfg.spd.shoulder + cfg.spd.elbow + cfg.spd.wrist);
    cyl.spd.p.r = vpd.x;
    cyl.spd.z = vpd.y;
    return cyl;
}


Vector3 direct(const Configuration& cfg){
    return fromInternal(toVec(fromConfiguration(cfg)));
}

MovingPoint<Vector3> direct(const MovingPoint<Configuration>& cfg){
    return fromInternal(toVec(fromConfiguration(cfg)));
}
Configuration inverse(const Vector3& vec){
    return toConfiguration(toCyl(toInternal(vec)));
}
MovingPoint<Configuration> inverse(const MovingPoint<Vector3>& vec){
    return toConfiguration(toCyl(toInternal(vec)));
}

// ┏┓ ┏━╸╺━┓╻┏━╸┏━┓
// ┣┻┓┣╸ ┏━┛┃┣╸ ┣┳┛
// ┗━┛┗━╸┗━╸╹┗━╸╹┗╸

Bezier1::Bezier1(Vector3 p0, Vector3 p1){
    this->p0 = p0;
    this->p1 = p1;
}

Vector3 Bezier1::position(double time) {
    double time_p = time * time * (3 - 2 * time);
    return (1 - time_p) * p0 + time_p * p1;
}

Vector3 Bezier1::speed(double time) {
    return 6 * time * (1 - time) * (p1 - p0);
}

void Bezier1::move(Matrix3 mat, Vector3 vec) {
    p0 = mat * p0 + vec;
    p1 = mat * p1 + vec;
}

Bezier2::Bezier2(Vector3 p0, Vector3 p1, Vector3 p2){
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
}

Vector3 Bezier2::position(double time) {
    double time_p = time * time * (3 - 2 * time);
    return (1 - time_p) * (1 - time_p) * p0
        + 2 * time_p * (1 - time_p) * p1
        + time_p * time_p * p2;
}

Vector3 Bezier2::speed(double time) {
    double time_p = time * time * (3 - 2 * time);
    return (12 * time * (1 - time)) * (
            (1 - time_p) * p0
            + (2 * time_p - 1) * p1
            - time_p * p2
        );
}

void Bezier2::move(Matrix3 mat, Vector3 vec) {
    p0 = mat * p0 + vec;
    p1 = mat * p1 + vec;
    p2 = mat * p2 + vec;
}

Bezier3::Bezier3(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3) :
    a(p0, p1, p2), b(p1, p2, p3) {}

Vector3 Bezier3::position(double time) {
    return (1 - time) * a.position(time) + time * b.position(time);
}

Vector3 Bezier3::speed(double time) {
    return - a.position(time) + (1 - time) * a.speed(time)
        + b.position(time) + time * b.speed(time);
}

void Bezier3::move(Matrix3 mat, Vector3 vec) {
    a.move(mat, vec);
    b.move(mat, vec);
}





