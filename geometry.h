#pragma once
#include <cassert>
#include <ostream>
#include <memory>
#include <vector>
#include <type_traits>
#include <cmath>
// all distances in mm, all times in seconds.

using uint = unsigned int;

// ┏━╸┏━┓┏┓╻┏━┓╺┳╸┏━┓┏┓╻╺┳╸┏━┓
// ┃  ┃ ┃┃┗┫┗━┓ ┃ ┣━┫┃┗┫ ┃ ┗━┓
// ┗━╸┗━┛╹ ╹┗━┛ ╹ ╹ ╹╹ ╹ ╹ ┗━┛

extern double l1; // mm
extern double l2; // mm
extern double epsilon;
extern double pi;

// ┏━┓╺┳╸┏━┓╻ ╻┏━╸╺┳╸
// ┗━┓ ┃ ┣┳┛┃ ┃┃   ┃ 
// ┗━┛ ╹ ╹┗╸┗━┛┗━╸ ╹ 

struct Polar;
struct Vector3{
    double x,y,z;

    double dot(const Vector3& vec) {
        return x * vec.x + y * vec.y + z * vec.z;
    }

    Vector3& operator += (const Vector3& vec){
        x += vec.x; y += vec.y; z += vec.z;
        return *this;
    }

    Vector3& operator -= (const Vector3& vec){
        x -= vec.x; y -= vec.y; z -= vec.z;
        return *this;
    }

    Vector3& operator *= (double d){
        x *= d; y *= d, z *= d;
        return *this;
    }

    Vector3& operator /= (double d){
        x /= d; y /= d, z /= d;
        return *this;
    }

    double norm1() const {
        return std::abs(x) + std::abs(y) + std::abs(z);
    }

    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    double norm22() const {
        return x*x + y*y + z*z;
    }

    double dot(Vector3 vec) const {
        return x*vec.x + y*vec.y + z*vec.z;
    }

    Vector3 cross(Vector3 vec) const {
        return Vector3 {
            y*vec.z - z*vec.y,
            z*vec.x - x*vec.z,
            x*vec.y - y*vec.x
        };
    }

    bool operator ==(Vector3 vec) const{
        vec -= (*this);
        return vec.norm1() < epsilon;
    }
    Vector3 rotatez(double angle) const{
        return Vector3{x * cos(angle) + y * sin(angle),
                y * cos(angle) - x * sin(angle),z};
    }
    Polar toPolar()const ;
};

#define OPER(oper,param) inline Vector3 operator oper (Vector3 v, param p) { v oper##= p; return v;}
OPER(+,const Vector3&)
OPER(-,const Vector3&)
OPER(*,double)
OPER(/,double)
inline Vector3 operator -(Vector3 v){return v * -1;}
inline Vector3 operator *(double p, Vector3 v){return v *= p;}
inline std::ostream& operator <<(std::ostream& out, const Vector3& vec){
    return out << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}

struct Matrix3 {
    Vector3 a,b,c; // lines

    Vector3 operator * (const Vector3& vec) {
        return {a.dot(vec), b.dot(vec), c.dot(vec)};
    }
};


extern Vector3 robotpos;
extern double robotor;

struct Polar{
    double theta, phi, r;
    Vector3 toVec(){
        return Vector3{r * cos(phi) * cos(theta),
                r * cos(phi) * sin(theta),
                r * sin(phi)
                };
    }
};

inline std::ostream& operator <<(std::ostream& out, const Polar& pol){
    return out << "(" << pol.r << ":" << pol.phi << ":" << pol.theta << ")";
}


struct Configuration{
    double base, shoulder, elbow, wrist;
};
inline std::ostream& operator <<(std::ostream& out, const Configuration& cfg){
    return out << "Conf(" << cfg.base << ", " << cfg.shoulder
               << ", " << cfg.elbow << ", " << cfg.wrist << ")";
}
inline Configuration operator+(const Configuration& c1, const Configuration& c2) {
    Configuration ret;
    ret.base     = c1.base     + c2.base;
    ret.shoulder = c1.shoulder + c2.shoulder;
    ret.elbow    = c1.elbow    + c2.elbow;
    ret.wrist    = c1.wrist    + c2.wrist;
    return ret;
}
inline Configuration operator-(const Configuration& c1, const Configuration& c2) {
    Configuration ret;
    ret.base     = c1.base     - c2.base;
    ret.shoulder = c1.shoulder - c2.shoulder;
    ret.elbow    = c1.elbow    - c2.elbow;
    ret.wrist    = c1.wrist    - c2.wrist;
    return ret;
}

struct Plane {
    //x in Plane <=> ortho.dot(x) == off
    Vector3 ortho;
    Vector3 e1, e2;
    double off;

    Plane(Vector3 a, Vector3 b, Vector3 c) {
        ortho = getOrtho(a, b, c);
        off = ortho.dot(a);
        ortho /= ortho.norm();
        e1 = ortho.cross(a-b);
        e1 /= e1.norm();
        e2 = ortho.cross(e1);
        e2 /= e2.norm();
    }

    Vector3 proj2d(Vector3 v) {
        Vector3 onPlane = v + (off-ortho.dot(v))*ortho;
        //TODO: return `onPlane` in the basis e1, e2
    }

    static Vector3 getOrtho(Vector3 a, Vector3 b, Vector3 c) {
        return (a-b).cross(a-c);
    }
};

// ┏━╸╻ ╻┏┓╻┏━╸╺┳╸╻┏━┓┏┓╻┏━┓
// ┣╸ ┃ ┃┃┗┫┃   ┃ ┃┃ ┃┃┗┫┗━┓
// ╹  ┗━┛╹ ╹┗━╸ ╹ ╹┗━┛╹ ╹┗━┛

Vector3 toInternal(Vector3 vec);
Vector3 fromInternal(Vector3 vec);

Configuration toConfiguration(Polar pol);
Polar fromConfiguration(Configuration cfg);

Vector3 direct(Configuration cfg);
Configuration inverse(Vector3 vec);

// derivatives;

Polar toPolarD(Vector3 vec, Vector3 vecd);
Vector3 fromPolarD(Polar pol, Polar pold);

Configuration toConfigurationD(Polar pol, Polar pold);
Configuration toConfigurationDBis(Configuration cfg, Polar pol, Polar pold);
Polar fromConfigurationD(Configuration cfg, Configuration cfgd);

Vector3 directD(Configuration cfg, Configuration cfgd);
Configuration inverseD(Vector3 vec, Vector3 vecd);
Configuration inverseDBis(Configuration cfg, Vector3 vec, Vector3 vecd);

// ╺┳╸┏━┓┏━┓ ┏┓┏━╸┏━╸╺┳╸┏━┓┏━┓╻ ╻
//  ┃ ┣┳┛┣━┫  ┃┣╸ ┃   ┃ ┃ ┃┣┳┛┗┳┛
//  ╹ ╹┗╸╹ ╹┗━┛┗━╸┗━╸ ╹ ┗━┛╹┗╸ ╹ 

class Trajectory {
public:
    class TrajectoryI{
        friend Trajectory;
    public:
        virtual Vector3 position(double time) = 0;
        virtual Vector3 speed(double time){
            if (1 - time <= 2*epsilon) {
                return (position(time) - position(time - epsilon))/epsilon;
            }
            if(time <= 2* epsilon){
                return (position(time + epsilon) - position(time))/epsilon;
            }
            return (position(time + epsilon) - position(time-epsilon))/(2*epsilon);
        }
        virtual void move(Matrix3, Vector3) = 0;
    };
private:
    std::shared_ptr<TrajectoryI> traj;
public:
    Trajectory() = delete;
    template<typename T, typename = std::enable_if_t<std::is_base_of<TrajectoryI, T>::value>>
    Trajectory(T t) : traj(new T(std::move(t))){}

    inline Vector3 position(double time){
        assert(time >= 0 and time <= 1);
        return traj->position(time);
    }
    inline Vector3 speed(double time){
        assert(time >= 0 and time <= 1);
        return traj->speed(time);
    }
    inline void move(Matrix3 a, Vector3 b) {
        traj->move(a, b);
    }
    inline Vector3 operator()(double time){return position(time);}
    Vector3 speedToFollow(double time, Vector3 currentPos);
};



template<size_t N> // TODO change that.
Trajectory equiConcat(std::array<Trajectory,N> l){
    static_assert(N>0);
    for(uint i = 0 ; i < N-1 ; ++i){
        assert(l[i](1) == l[i+1](0));
    }
    struct Impl : Trajectory::TrajectoryI{
        std::array<Trajectory, N> trajs;
        Impl(std::array<Trajectory, N> l) : trajs(l){}
        Vector3 position(double time){
            double stime = time *N;
            uint val = uint(stime);
            if (val == N) --val;
            return trajs[val](stime - val);
        }
        Vector3 speed(double time){
            double stime = time *N;
            uint val = uint(stime);
            if (val == N) --val;
            return trajs[val].speed(stime - val) * N;
        }
        void move(Matrix3 mat, Vector3 vec) {
            for(auto &traj : trajs) traj.move(mat, vec);
        }
    };
    return Impl(l);
}
Trajectory equiConcatv(std::vector<Trajectory> list);
inline Trajectory operator-(Trajectory t1, Trajectory t2){return equiConcat<2>({{t1,t2}});}



// ┏┓ ┏━╸╺━┓╻┏━╸┏━┓
// ┣┻┓┣╸ ┏━┛┃┣╸ ┣┳┛
// ┗━┛┗━╸┗━╸╹┗━╸╹┗╸

class Bezier1 : public Trajectory::TrajectoryI {
    private:
        Vector3 p0, p1;
    public:
        Vector3 position(double);
        Vector3 speed(double);
        void move(Matrix3, Vector3);
        Bezier1(Vector3, Vector3);
};

class Bezier2 : public Trajectory::TrajectoryI {
    private:
        Vector3 p0, p1, p2;
    public:
        Vector3 position(double);
        Vector3 speed(double);
        void move(Matrix3, Vector3);
        Bezier2(Vector3, Vector3, Vector3);
};

class Bezier3 : public Trajectory::TrajectoryI {
    private:
        Bezier2 a, b;
    public:
        Vector3 position(double);
        Vector3 speed(double);
        void move(Matrix3, Vector3);
        Bezier3(Vector3, Vector3, Vector3, Vector3);
};





