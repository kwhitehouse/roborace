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

        // vector<Polygon*> createConvexHulls(const vector<Polygon*> &obstacles);

        // void removeHullsPassed(vector<Polygon*> &obstacles);

        // double cross(coord &p, coord &q, coord &r);

        // bool segmentsIntersect(const coord &, const coord &, const coord &, const coord &);

        // vector<coord> visibleVertices(const vector <pair<coord, coord> > &, vector<coord>, const coord &);

        // std::map<coord, vector<coord> > constructVisibilityGraph(const std::vector<Polygon *> &obstacles,const Polygon* &boundary, const coord &start, const coord &goal);

        // vector<coord*> dijkstra(coord &source, map<coord, vector<coord> > &visibility_graph, coord &goal);

        // void pathPlan(vector<coord> &path, double &angle, double &dist);

        // vector<float> getPathInfo(coord &c1, coord &c2);

        coord goal;
        coord curr_pos;
        bool finished;
};

#endif
