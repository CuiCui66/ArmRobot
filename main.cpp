#include <iostream>
#include <unistd.h>
#include <ctime>
#include "penWriter.h"
#include "geometry.h"
#include "control.h"

#define FONT "GreatVibes"
#define DURATION 3

using namespace std;

#define FOLLOW_TRAJ(traj) \
    beg = clock(); clk = clock(); \
    end = beg + DURATION * CLOCKS_PER_SEC; \
    while(clk < end) { \
        bot.applyConfigurationSpeed(conf); \
        mp.pos = direct(bot.configuration()); \
        mp.spd = traj.follow(mp.pos, clk, beg, end); \
        conf = inverse(mp).spd; \
        clk = clock(); \
    }

#define HGH 35

int main() {
    Robot bot;
    Configuration conf;
    MovingPoint<Vector3> mp;
    clock_t beg, end, clk;
    Vector3 controls[8] = {
        {150,0,HGH}, {150,50,HGH}, {250,50,30}, {350,50,22},
        {350,0,22}, {350,-50,22}, {250,-50,30}, {150,-50,HGH}
    };
    Bezier3 b1(controls[0], controls[1], controls[1], controls[2]);
    Bezier3 b2(controls[2], controls[3], controls[3], controls[4]);
    Bezier3 b3(controls[4], controls[5], controls[5], controls[6]);
    Bezier3 b4(controls[6], controls[7], controls[7], controls[0]);
    Trajectory t1(b1);
    Trajectory t2(b2);
    Trajectory t3(b3);
    Trajectory t4(b4);

    cout << "Initialization ..." << endl;
    bot.init();
    cout << "Initialisation finished" << endl;

    conf = inverse(controls[0]);
    bot.point(conf);

    Bezier3 b(controls[1], controls[3], controls[7], controls[5]);
    Trajectory t(b);
    FOLLOW_TRAJ(t);

    // while(true) {
    //    FOLLOW_TRAJ(t1);
    //    FOLLOW_TRAJ(t2);
    //    FOLLOW_TRAJ(t3);
    //    FOLLOW_TRAJ(t4);
    // }
    bot.stop();

    // p1.x = 300; p1.y = 150; p1.z = 200;
    // Bezier1 bez(mp.pos, p1);
    // Trajectory traj(bez);

    // cout << "pause ..." << endl;
    // cin >> beg;
    // beg = clock(); clk = clock();
    // end = beg + duration * CLOCKS_PER_SEC;
    // while(clk < end) {
    //     bot.applyConfigurationSpeed(conf);
    //     mp.pos = direct(bot.configuration());
    //     mp.spd = traj.follow(mp.pos, clk, beg, end);
    //     conf = inverse(mp).spd;
    //     clk = clock();
    // }
    // bot.stop();

    // while(true) {
    //     cout << "X : ";
    //     cin >> p1.x;
    //     cout << "Y : ";
    //     cin >> p1.y;
    //     cout << "Z : ";
    //     cin >> p1.z;
    //     conf = inverse(p1);
    //     cout << "Conf : " << conf << endl;
    //     bot.point(conf);
    // }
    return 0;
}
