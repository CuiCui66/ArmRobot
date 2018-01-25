#pragma once
#include <cassert>
#include <ostream>
#include <memory>
#include <vector>
#include <type_traits>
#include <cmath>
// all distances in mm, all times in seconds, all angles in rad;

using uint = unsigned int;

// ┏━╸┏━┓┏┓╻┏━┓╺┳╸┏━┓┏┓╻╺┳╸┏━┓
// ┃  ┃ ┃┃┗┫┗━┓ ┃ ┣━┫┃┗┫ ┃ ┗━┓
// ┗━╸┗━┛╹ ╹┗━┛ ╹ ╹ ╹╹ ╹ ╹ ┗━┛

extern double l1; // mm
extern double l2; // mm
extern double l3; // mm
extern double epsilon;
extern double pi;



// ┏━┓╺┳╸┏━┓╻ ╻┏━╸╺┳╸
// ┗━┓ ┃ ┣┳┛┃ ┃┃   ┃
// ┗━┛ ╹ ╹┗╸┗━┛┗━╸ ╹

template<typename T>
struct MovingPoint{
    MovingPoint(T pos, T spd) : pos(pos), spd(spd){}
    MovingPoint(T pos): pos(pos), spd(0.0){
        static_assert(std::is_same<T,double>::value,"");
    }
    MovingPoint(){}
    T pos, spd;
};
template<typename T>
MovingPoint<T> mp(T pos, T spd){
    return MovingPoint<T>(pos,spd);
}

// few helper function:
inline MovingPoint<double> operator+(MovingPoint<double>d1, MovingPoint<double> d2){
    return mp(d1.pos + d2.pos,d1.spd + d2.spd);
}

inline MovingPoint<double> operator-(MovingPoint<double>d1, MovingPoint<double> d2){
    return mp(d1.pos - d2.pos,d1.spd - d2.spd);
}

inline MovingPoint<double> operator*(MovingPoint<double>d1, MovingPoint<double> d2){
    return mp(d1.pos * d2.pos,d1.spd * d2.pos + d1.pos * d2.spd);
}

inline MovingPoint<double> operator/(MovingPoint<double>d1, MovingPoint<double> d2){
    return mp(d1.pos / d2.pos, (d1.spd * d2.pos - d2.spd * d1.pos)/(d2.pos * d2.pos));
}

inline MovingPoint<double> sqrt(MovingPoint<double> d){
    return mp(std::sqrt(d.pos), d.spd  / (2*sqrt(d.pos)));
}

inline MovingPoint<double> acos(MovingPoint<double> d){
    return mp(acos(d.pos), - d.spd / sqrt(1 - d.pos*d.pos));
}
template<typename T>
inline std::ostream& operator <<(std::ostream& out, const MovingPoint<T>& mp){
    return out << "(p: " << mp.pos << ", v: " << mp.spd << ")";
}


//  ____  ____
// |___ \|  _ \
//   __) | | | |
//  / __/| |_| |
// |_____|____/


struct Vector2{
    double x,y;

    Vector2& operator += (const Vector2& vec){
        x += vec.x; y += vec.y;
        return *this;
    }

    Vector2& operator -= (const Vector2& vec){
        x -= vec.x; y -= vec.y;
        return *this;
    }

    Vector2& operator *= (double d){
        x *= d; y *= d;
        return *this;
    }

    Vector2& operator /= (double d){
        x /= d; y /= d;
        return *this;
    }

    double norm1() const {
        return std::abs(x) + std::abs(y);
    }

    double norm() const {
        return std::sqrt(x*x + y*y);
    }

    double norm22() const {
        return x*x + y*y;
    }
    double dot(const Vector2& vec) const{
        return x*vec.x + y * vec.y;
    }
    double det(const Vector2& vec) const{
        return x*vec.y - y * vec.x;
    }


    bool operator ==(Vector2 vec) const{
        vec -= (*this);
        return vec.norm1() < epsilon;
    }
    Vector2 rotate(double angle) const{
        return Vector2{x * cos(angle) + y * sin(angle),
                y * cos(angle) - x * sin(angle)};
    }
};

#define OPER(oper,param) inline Vector2 operator oper (Vector2 v, param p) { return v oper##= p;}
OPER(+,const Vector2&)
OPER(-,const Vector2&)
OPER(*,double)
OPER(/,double)
inline Vector2 operator -(Vector2 v){return v * -1;}
inline Vector2 operator *(double p, Vector2 v){return v *= p;}
inline std::ostream& operator <<(std::ostream& out, const Vector2& vec){
    return out << "(" << vec.x << ", " << vec.y << ")";
}


struct Polar2{
    double r,theta;
};

inline std::ostream& operator <<(std::ostream& out, const Polar2& pol){
    return out << "(" << pol.r << ":" << pol.theta << ")";
}


Polar2 toPol(const Vector2& vec);
MovingPoint<Polar2> toPol(const MovingPoint<Vector2>& vec);
Vector2 toVec(const Polar2& pol);
MovingPoint<Vector2> toVec(const MovingPoint<Polar2>& pol);


//  _____ ____
// |___ /|  _ \
//   |_ \| | | |
//  ___) | |_| |
// |____/|____/


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
    Vector2 lower() const{
        return Vector2{x,y};
    }
};

#define OPER3(oper,param) inline Vector3 operator oper (Vector3 v, param p) { return v oper##= p;}
OPER3(+,const Vector3&)
OPER3(-,const Vector3&)
OPER3(*,double)
OPER3(/,double)
inline Vector3 operator -(Vector3 v){return v * -1;}
inline Vector3 operator *(double p, Vector3 v){return v *= p;}
inline std::ostream& operator <<(std::ostream& out, const Vector3& vec){
    return out << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}

struct Matrix3 {
    Vector3 a,b,c; // lines

    Matrix3 operator * (double d) {
        return {a * d, b * d, c * d};
    }

    Vector3 operator * (const Vector3& vec) {
        return {a.dot(vec), b.dot(vec), c.dot(vec)};
    }
};


extern Vector3 robotpos;
extern Vector2 O1pos;
extern double robotor;

struct Cylindrical{
    Polar2 p;
    double z;
    Cylindrical& operator += (const Vector2& vec){
        p.r += vec.x; z += vec.y;
        return *this;
    }

    Cylindrical& operator -= (const Vector2& vec){
        p.r -= vec.x; z -= vec.y;
        return *this;
    }

    Cylindrical& operator *= (double d){
        p.r *= d; z *= d;
        return *this;
    }

    Vector2 vertPlane() const{
        return Vector2{p.r,z};
    }
};

#define OPERC(oper,param) inline Cylindrical operator oper (Cylindrical v, param p)\
    { return v oper##= p;}
OPERC(+,const Vector2&)
OPERC(-,const Vector2&)
OPERC(*,double)


inline std::ostream& operator <<(std::ostream& out, const Cylindrical& cyl){
    return out << "(" << cyl.p << ", " << cyl.z << ")";
}


struct Polar3{
    Polar2 p;
    double theta;
};

inline std::ostream& operator <<(std::ostream& out, const Polar3& pol){
    return out << "(" << pol.p << ":" << pol.theta << ")";
}

Vector3 toVec(const Cylindrical& cyl);
MovingPoint<Vector3> toVec(const MovingPoint<Cylindrical>& cyl);
Vector3 toVec(const Polar3& pol);
MovingPoint<Vector3> toVec(const MovingPoint<Polar3>& pol);

Polar3 toPol(const Vector3& vec);
MovingPoint<Polar3> toPol(const MovingPoint<Vector3>& vec);
Polar3 toPol(const Cylindrical& vec);
MovingPoint<Polar3> toPol(const MovingPoint<Cylindrical>& vec);

Cylindrical toCyl(const Vector3& vec);
MovingPoint<Cylindrical> toCyl(const MovingPoint<Vector3>& vec);
Cylindrical toCyl(const Polar3& vec);
MovingPoint<Cylindrical> toCyl(const MovingPoint<Polar3>& vec);


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


//   ____             __ _                       _   _
//  / ___|___  _ __  / _(_) __ _ _   _ _ __ __ _| |_(_) ___  _ __
// | |   / _ \| '_ \| |_| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \
// | |__| (_) | | | |  _| | (_| | |_| | | | (_| | |_| | (_) | | | |
//  \____\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
//                         |___/



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


// ┏━╸╻ ╻┏┓╻┏━╸╺┳╸╻┏━┓┏┓╻┏━┓
// ┣╸ ┃ ┃┃┗┫┃   ┃ ┃┃ ┃┃┗┫┗━┓
// ╹  ┗━┛╹ ╹┗━╸ ╹ ╹┗━┛╹ ╹┗━┛

// changing from global to O
Vector3 toInternal(const Vector3& vec);
Vector3 fromInternal(const Vector3& vec);

// changing between configuration and cylindrical from O1
Configuration toConfiguration(Cylindrical cyl);
Cylindrical fromConfiguration(const Configuration& cfg);

Vector3 direct(const Configuration& cfg);
Configuration inverse(const Vector3& vec);


MovingPoint<Vector3> toInternal(const MovingPoint<Vector3>& vec);
MovingPoint<Vector3> fromInternal(const MovingPoint<Vector3>& vec);

MovingPoint<Configuration> toConfiguration(MovingPoint<Cylindrical> pol);
MovingPoint<Cylindrical> fromConfiguration(const MovingPoint<Configuration>& cfg);

MovingPoint<Vector3> direct(const MovingPoint<Configuration>& cfg);
MovingPoint<Configuration> inverse(const MovingPoint<Vector3>& vec);

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
    static_assert(N>0,"");
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
inline Trajectory equiConcatv(std::vector<Trajectory> l){
    uint N = l.size();
    for(uint i = 0 ; i < N-1; ++i){
        assert(l[i](1) == l[i+1](0));
    }
    struct Impl : Trajectory::TrajectoryI{
        std::vector<Trajectory> trajs;
        Impl(std::vector<Trajectory> l) : trajs(l){}
        Vector3 position(double time){
            uint N = trajs.size();
            double stime = time * N;
            uint val = uint(stime);
            if (val == N) --val;
            return trajs[val](stime - val);
        }
        Vector3 speed(double time){
            uint N = trajs.size();
            double stime = time * N;
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





