#include <iostream>
#include "penWriter.h"
#include "geometry.h"

#define FONT "GreatVibes"

using namespace std;

int main(){
    cout << "hello world" << endl;

    Vector3 i{-89,43,96};
    Vector3 id{44562,145626,45};
    Cylindrical c{Polar2{70,0},O1pos.y-l3};
    Cylindrical cd{Polar2{4,7},100};
    auto cfg = toConfiguration(mp(c,cd));
    cout << direct(inverse(mp(i,id))) << endl;
    return 0;
}
