#include <iostream>
#include <map>
#include "geometry.h"

#define FONT "GreatVibes"

using namespace std;

int main(){
    map<char, vector<Trajectory::TrajectoryI>> font;
    FILE *fontfile = fopen("./fonts/" FONT ".font", "r");
    int nbChar; fscanf(fontfile, "%d", &nbChar);
    for(int iChar = 0; iChar < nbChar; ++iChar) {
        char l; int nbTraj; fscanf(fontfile, " %c%d", &l, &nbTraj);
        font[l] = vector<Trajectory::TrajectoryI>();
        for(int iTraj = 0; iTraj < nbTraj; ++iTraj) {
            int nbPts; fscanf(fontfile, "%d", &nbPts);
            vector<Vector3> p;
            for(int iPt = 0; iPt < nbPts; ++iPt) {
                float x, y; fscanf(fontfile, "%f%f", &x, &y);
                p.push_back({x, y, 0});
            }
            switch(nbPts) {
                case 4:
                    font[l].push_back(Bezier3(p[0], p[1], p[2], p[3]));
                    break;
                default:
                    fprintf(stderr, "Error in the font file: Unsupported trajectory\n");
            }
        }
    }
    fclose(fontfile);
    return 0;
}
