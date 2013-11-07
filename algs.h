#ifndef __ALGS_H__
#define __ALGS_H__

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
	algs(coord &start, coord &gol): finished(false), curr_pos(start), goal(gol){}

	~algs();

	void algs::growObstacles(vector<Polygon*> &obstacles);

	void algs::replaceWithConvexHulls(vector<Polygon*> &obstacles);

	void algs::removeHullsPassed(vector<Polygon*> &obstacles);

	void algs::determineReachableCoords(coord & perceived_pos, vector<Polygon*> &obstacles, vector<coord> &d_coords);

	void algs::dijkstra(vector<Polygon*> &obstacles, vector<coord> &path);

	void algs::pathPlan(vector<coord> &path, double &angle, double &dist);

	void algs::renderVisibilityGraph(int map_id, vector<coord> &path, Polygon &boundary, vector<Polygon*> &o_obs, vector<Polygon*> &g_obs);

	coord goal;
	coord curr_pos;
	bool finished;
	vector<bid_edge> potential_paths;
};

#endif