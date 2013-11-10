#ifndef __ALGS_H__
#define __ALGS_H__

#include <map>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"


struct bid_edge //bi-directional edge
{
    coord _a;
    coord _b;
};

class algs
{
public:
	algs(const coord &start, const coord &gol): goal(gol), curr_pos(start), finished(false) {}

	void growObstacles(vector<Polygon*> &obstacles);

	void replaceWithConvexHulls(vector<Polygon*> &obstacles);

	void removeHullsPassed(vector<Polygon*> &obstacles);
    
    double cross(coord &p, coord &q, coord &r);
    
    
    std::map<coord, vector<coord> > visibilityGraph(const std::vector<Polygon *> &obstacles);
    
    std::vector<coord> visibleVertices(const coord &point, const std::vector<Polygon*> &obstacles);
   
    bool visible(const coord &vertex);

    void dijkstra(map<coord, vector<coord> > &visibility_graph, const coord &source);

	void pathPlan(vector<coord> &path, double &angle, double &dist);

	void renderVisibilityGraph(int map_id, vector<coord> &path, Polygon &boundary, vector<Polygon*> &o_obs, vector<Polygon*> &g_obs);

	coord goal;
	coord curr_pos;
	bool finished;
	vector<bid_edge> potential_paths;
};

#endif
