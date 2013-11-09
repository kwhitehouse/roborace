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
	algs(coord &start, coord &gol): finished(false), curr_pos(start), goal(gol){}

	~algs();

	void growObstacles(vector<Polygon*> &obstacles);

	void replaceWithConvexHulls(vector<Polygon*> &obstacles);

	void removeHullsPassed(vector<Polygon*> &obstacles);
    
    double cross(coord &p, coord &q, coord &r);
    
    bool sortByAngles(std::pair<coord, std::pair<double, double>> pair1, std::pair<coord, std::pair<double, double>> pair2);
    
    bool sortByDistances(std::pair<std::pair<coord, coord>, double> pair1, std::pair<std::pair<coord, coord>, double> pair2);
    
    std::map<coord, vector<coord>> visibilityGraph(const std::vector<Polygon *> &obstacles);
    
    std::vector<coord> visibleVertices(const coord &point, const std::vector<Polygon*> &obstacles);
   
    bool visible(const coord &vertex);

	void dijkstra(vector<Polygon*> &obstacles, vector<coord> &path);

	void pathPlan(vector<coord> &path, double &angle, double &dist);

	void renderVisibilityGraph(int map_id, vector<coord> &path, Polygon &boundary, vector<Polygon*> &o_obs, vector<Polygon*> &g_obs);

	coord goal;
	coord curr_pos;
	bool finished;
	vector<bid_edge> potential_paths;
};

#endif