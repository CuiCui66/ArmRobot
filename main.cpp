#include <iostream>
#include "geometry.h"

using namespace std;

int main(){
    cout << "hello world" << endl;

    Vector3 v = direct(inverse(Vector3{1,2,3.4562789}));
    cout << v.x << " " << v.y << " " << v.z << endl;
    return 0;
}
