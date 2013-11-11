#ifndef __ALGS_H__
#define __ALGS_H__

#include <map>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"


class algs
{
    public:
        algs(const coord &start, const coord &gol): goal(gol), curr_pos(start), finished(false) {}

        vector<Polygon*> growObstacles(vector<Polygon*> &obstacles);

        void replaceWithConvexHulls(vector<Polygon*> &obstacles);

        void removeHullsPassed(vector<Polygon*> &obstacles);

        double cross(coord &p, coord &q, coord &r);

        bool segmentsIntersect(const coord &, const coord &, const coord &, const coord &);

        std::map<coord, vector<coord> > constructVisibilityGraph(const std::vector<Polygon *> &obstacles);

        void dijkstra(map<coord, vector<coord> > &visibility_graph, const coord &source);

        void pathPlan(vector<coord> &path, double &angle, double &dist);

        coord goal;
        coord curr_pos;
        bool finished;
};

#endif
