#include "geometry.h"


double l1 = 100.0; // mm
double l2 = 100.0; // mm
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
    return acos((c1*c1 + c2*c2 - c3*c3)/(2 * c1 * c2));
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
    assert(std::abs(2*pi - cfg.shoulder - cfg.elbow - cfg.wrist) < epsilon);
    Polar pol;
    pol.r =  triangleThirdLength(l1, l2, cfg.elbow);
    pol.phi = cfg.shoulder - triangleAngle(l1, pol.r, l2);
    pol.theta = cfg.base;
    return pol;
}

Vector3 direct(Configuration cfg){
    return fromInternal(fromConfiguration(cfg).toVec());
}
Configuration inverse(Vector3 vec){
    return toConfiguration(toInternal(vec).toPolar());
}

