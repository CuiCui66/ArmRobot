#include <iostream>
#include "penWriter.h"

#define FONT "GreatVibes"

using namespace std;

int main(){
    PenWriter pw(100, FONT);
    pw.write();
    return 0;
}
