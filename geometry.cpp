#include "geometry.h"
#include <iostream>


double l1 = 199.25; // mm
double l2 = 200.68; // mm
double epsilon = 0.00001;
double pi = std::acos(-1);
Vector3 robotpos{0,0,100.0};
double robotor = 0.0;

using namespace std;
Vector3 toInternal(Vector3 vec){
    return (vec - robotpos).rotatez(-robotor);
}
Vector3 fromInternal(Vector3 vec){
    return (vec + robotpos).rotatez(robotor);
}

Polar Vector3::toPolar() const{
    return Polar{
        atan2(y,x),
        atan2(z,sqrt(x*x + y*y)),
        norm()
    };
}

double triangleAngle(double c1, double c2, double c3){
    double in = (c1*c1 + c2*c2 - c3*c3)/(2 * c1 * c2);
    assert(abs(in) <= 1);
    return acos(in);
}
double triangleThirdLength(double c1, double c2, double angle){
    return sqrt(c1*c1 + c2*c2 - 2*c1*c2 * cos(angle));
}

Configuration toConfiguration(Polar pol){
    double shoulder = triangleAngle(l1, pol.r, l2) + pol.phi;
    double elbow = triangleAngle(l1, l2, pol.r);
    return Configuration{pol.theta, shoulder,elbow, 2*pi-shoulder-elbow};
}
Polar fromConfiguration(Configuration cfg){
    if(!(abs(2*pi - cfg.shoulder - cfg.elbow - cfg.wrist) < epsilon)){
        cerr << "Warning in " << cfg << " invariant "
             << std::abs(2*pi - cfg.shoulder - cfg.elbow - cfg.wrist)
             << " >= epsilon (" << epsilon << ")" << endl;
    }
    assert(std::abs(2*pi - cfg.shoulder - cfg.elbow - cfg.wrist) < epsilon);
    Polar pol;
    pol.r =  triangleThirdLength(l1, l2, cfg.elbow);
    pol.phi = cfg.shoulder - atan2(l2*cos(cfg.elbow) + l1,l2*sin(cfg.elbow));
    pol.theta = cfg.base;
    return pol;
}

Vector3 direct(Configuration cfg){
    return fromInternal(fromConfiguration(cfg).toVec());
}
Configuration inverse(Vector3 vec){
    return toConfiguration(toInternal(vec).toPolar());
}

// ┏━╸╻┏┓╻┏━╸┏┳┓┏━┓╺┳╸╻┏━╸┏━┓
// ┃  ┃┃┗┫┣╸ ┃┃┃┣━┫ ┃ ┃┃  ┗━┓
// ┗━╸╹╹ ╹┗━╸╹ ╹╹ ╹ ╹ ╹┗━╸┗━┛



double atan2D(double x, double y, double xd, double yd){
    return(yd* x - xd * y)/ (x*x + y*y);
}

Polar toPolarD(Vector3 vec, Vector3 vecd){
    auto [x,y,z] = vec;
    auto [xd,yd,zd] = vecd;
    auto r0 = hypot(x, y);
    auto r0d = (x * xd+ y * yd) / r0;
    Polar pol;
    pol.theta = atan2D(x,y,xd,yd);
    pol.phi = atan2D(r0, z, r0d, zd);
    pol.r = (x*xd + y * yd + z *zd) / vec.norm();
    return pol;
}
Vector3 fromPolarD(Polar pol, Polar pold){
    auto [theta,phi,r] = pol;
    auto [thetad,phid,rd] = pold;
    Vector3 vec;
    vec.x = rd * cos(phi) * cos(theta)
        + r * phid * (-sin(phi)) * cos(theta)
        + r * cos(phi) * thetad * (-sin(theta));
    vec.y = rd * cos(phi) * sin(theta)
        + r * phid * (-sin(phi)) * sin(theta)
        + r * cos(phi) * thetad * cos(theta);
    vec.z = rd * sin(phi) + r * phid * cos(phi);
    return vec;
}

double triangleAngled1(double c1, double c2, double c3, double c1d){
    double up = (c1*c1 + c2*c2 - c3*c3);
    double down = 2 * c1 * c2;
    double in = up/down;
    assert(abs(in) < 1);
    double ind = (2*c1*c1d * 2 *c1 *c2 - up * 2 * c1d * c2)/ (down * down);
    return ind * 1/sqrt(1-in*in);
}
double triangleAngled3(double c1, double c2, double c3, double c3d){
    double up = (c1*c1 + c2*c2 - c3*c3);
    double down = 2 * c1 * c2;
    double in = up/down;
    double ind = (-2 * c3 * c3d) / down;
    cout << ind << endl;
    return ind * (-1)/sqrt(1-in*in);
}

double triangleThirdLengthD(double c1, double c2, double angle, double angled){
    double in =(c1*c1 + c2*c2 - 2*c1*c2 * cos(angle));
    double ind = -2 * c1 * c2 * angled * (- sin(angle));
    return ind/(2*sqrt(in));
}


Configuration toConfigurationD(Polar pol, Polar pold){
    Configuration cfg;
    cfg.base = pold.theta;
    cfg.shoulder = triangleAngled1(pol.r, l1, l2, pold.r) + pold.phi;
    cfg.elbow = triangleAngled3(l1, l2, pol.r, pold.r);
    cfg.wrist = - cfg.shoulder - cfg.elbow;
    return cfg;
}
Polar fromConfigurationD(Configuration cfg, Configuration cfgd){
    auto [base,shoulder,elbow,wrist] = cfg;
    auto [based,shoulderd,elbowd,wristd] = cfgd;
    Polar pol;
    pol.r = triangleThirdLengthD(l1, l2, elbow, elbowd);
    pol.phi = shoulderd - atan2D(l2*cos(elbow) +l1, l2 * sin(elbow),
                                 l2*elbowd*(-sin(elbow)), l2 * elbowd * cos(elbow));
    pol.theta = based;
    return pol;
}

Vector3 directD(Configuration cfg, Configuration cfgd){
    Polar pol = fromConfiguration(cfg);
    Polar pold = fromConfigurationD(cfg, cfgd);
    return fromPolarD(pol, pold);
}

Configuration inverseD(Vector3 vec, Vector3 vecd){
    vec = toInternal(vec);
    Polar pol = vec.toPolar();
    Polar pold = toPolarD(vec, vecd);
    return toConfigurationD(pol, pold);
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
    return (1 - time_p) * this->p0 + time_p * this->p1;
}

Vector3 Bezier1::speed(double time) {
    return 6 * time * (1 - time) * (this->p1 - this->p0);
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
    return (1 - time_p) * (1 - time_p) * this->p0
        + 2 * time_p * (1 - time_p) * this->p1
        + time_p * time_p * this->p2;
}

Vector3 Bezier2::speed(double time) {
    double time_p = time * time * (3 - 2 * time);
    return (12 * time * (1 - time)) * (
            (1 - time_p) * this->p0
            + (2 * time_p - 1) * this->p1
            - time_p * this->p2
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





