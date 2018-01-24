#include <iostream>
#include "geometry.h"

using namespace std;

int main(){
    cout << "hello world" << endl;

    Vector3 i{1,1,55};
    Vector3 id{4,5.34,6};
    Configuration cfg = inverse(i);
    Polar p {1,0.1,12};
    Polar pd {1,2,3};
    cout << toConfigurationD(p, pd) << endl;
    cout << fromConfigurationD(toConfiguration(p),toConfigurationD(p,pd)) << endl;
    return 0;
}
