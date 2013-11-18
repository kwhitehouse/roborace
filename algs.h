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

        vector<Polygon*> createReflections(vector<Polygon*> &obstacles);

        vector<Polygon*> createConvexHulls(const vector<Polygon*> &obstacles);

        void removeHullsPassed(vector<Polygon*> &obstacles);

        double cross(coord &p, coord &q, coord &r);

        bool segmentsIntersect(const coord &, const coord &, const coord &, const coord &);

        vector<coord> visibleVertices(const vector <pair<coord, coord> > &, vector<coord>, const coord &);

        std::map<coord, vector<coord> > constructVisibilityGraph(const std::vector<Polygon *> &obstacles, const coord &start, const coord &goal);

        vector<coord*> dijkstra(coord &source, map<coord, vector<coord> > &visibility_graph, coord &goal);

        void pathPlan(vector<coord> &path, double &angle, double &dist);

        coord goal;
        coord curr_pos;
        bool finished;
};

#endif
