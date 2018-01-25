#include "penWriter.h"

#define ever (;;)

void PenWriter::loadFont(std::string fontname) {
    FILE *fontfile = fopen(("./fonts/" + fontname + ".font").c_str(), "r");
    int nbChar; fscanf(fontfile, "%d", &nbChar);
    for(int iChar = 0; iChar < nbChar; ++iChar) {
        char l; int nbTraj; fscanf(fontfile, " %c%d", &l, &nbTraj);
        font[l] = {};
        for(int iTraj = 0; iTraj < nbTraj; ++iTraj) {
            int nbPts; fscanf(fontfile, "%d", &nbPts);
            std::vector<Vector3> p;
            for(int iPt = 0; iPt < nbPts; ++iPt) {
                float x, y; fscanf(fontfile, "%f%f", &x, &y);
                p.push_back({x, y, 0});
            }
            switch(nbPts) {
                case 2:
                    font[l].push_back(Bezier1(p[0], p[1]));
                    break;
                case 3:
                    font[l].push_back(Bezier2(p[0], p[1], p[2]));
                    break;
                case 4:
                    font[l].push_back(Bezier3(p[0], p[1], p[2], p[3]));
                    break;
                default:
                    fprintf(stderr, "Error in the font file: Unsupported trajectory\n");
            }
        }
    }
    fclose(fontfile);
}

void PenWriter::setSize(double size) {
    assert(size >= 10 && size <= 1000);
    this->size = size;
}

void PenWriter::write() {
    char buffer[42]; // use string ?
    double offset = 0;
    Vector3 last = {0, 0, 0};
    // TODO move the robot to WRITING_START
    for ever {
        scanf(" %s", buffer);
        std::string word = buffer;
        for(auto l: word) {
            auto traj = getTraj(l, last);
            last = traj(1) - WRITING_DIR;
            traj.move(Matrix3(
                    {{ROT, -ROT, 0}, // TODO add rotation to follow WRITING_DIR
                     {ROT, ROT, 0},
                     {0, 0, 1}}) * size,
                    PAPER_START + WRITING_DIR * offset * size);
            // TODO make robot do trajectory traj
            ++offset;
        }
        ++offset;
    }
}

Trajectory PenWriter::getTraj(char letter, Vector3 last) {
    auto svg = font[letter];
    std::vector<Trajectory> path;
    for(auto traj: svg) {
        if(!(traj(0) == last))
            path.push_back(Bezier3(
                        last,
                        last + RISE,
                        traj(0) + RISE,
                        traj(0)));
        path.push_back(traj);
        last = traj(1);
    }
    return equiConcatv(path);
}

PenWriter::PenWriter(double size, std::string fontname) {
    setSize(size);
    loadFont(fontname);
}





