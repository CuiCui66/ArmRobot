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
        return std::hypot(x, y, z);
    }

    double norm22() const {
        return x*x + y*y + z*z;
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
inline std::ostream& operator <<(std::ostream& out, const Vector3& vec){
    return out << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}


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
    protected:
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
    };
private:
    std::shared_ptr<TrajectoryI> traj;
public:
    Trajectory() = delete;
    template<typename T, typename = std::enable_if_t<std::is_base_of_v<TrajectoryI, T>>>
    Trajectory(T t) : traj(new T(std::move(t))){}

    inline Vector3 position(double time){
        assert(time >= 0 and time <= 1);
        return traj->position(time);
    }
    inline Vector3 speed(double time){
        assert(time >= 0 and time <= 1);
        return traj->speed(time);
    }
    inline Vector3 operator()(double time){return position(time);}
    Vector3 speedToFollow(double time, Vector3 currentPos);


    // basic ones :
    static Trajectory line(Vector3 a, Vector3 b);
    template<typename F> // F must have signature double -> Vector3;
    static Trajectory fromPos(F f){
        struct Impl : TrajectoryI{
            F f;
            Vector3 position(double time){ return f(time);}
        };
        return Impl{f};
    }
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
    };
    return Impl(l);
}
Trajectory equiConcatv(std::vector<Trajectory> list);
inline Trajectory operator-(Trajectory t1, Trajectory t2){return equiConcat<2>({{t1,t2}});}








