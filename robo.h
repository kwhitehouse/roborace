#ifndef __ROBO_H__
#define __ROBO_H__

#include <map>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "robo.h"
#include "bisc.h"


class robo
{
    public:
        robo(const coord &start, const coord &gol): goal(gol), curr_pos(start), finished(false) {}

        int init(char *device);

        void moveNextCoord(vector<float> dest);

        int disconnect();

        coord goal;
        coord curr_pos;
        bool finished;
};

#endif
