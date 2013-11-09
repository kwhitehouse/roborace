
#include "algs.h"
#include <vector>
#include <map>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"

//All functions have have access to:
//--// coord goal;
//--// coord curr_pos;
//--// bool finished;
//--// vector<bid_edge> potential_paths;


// Given the initial vector of polygons (obstacles), grow each one
// individually (all the way around) by some pretermined region
// in order to determine the workspace of the robot without collisions
// Inputs:
//   obstacles:  Original polygons consisting of the coordinates
//				 which make up their vertices
// Outputs:
//   obstacles:  The same polygons with coordinates added to each.
//   			 polygon accounting for their growth regions
void algs::growObstacles(vector<Polygon*> &obstacles)
{
	//Emily
	//code goes here
}

// Removes unesseary coordinates from vector of polygons (obstacles)
// so that all which remains per polygon is the outtermost vertices
// that compose its covex hull and will therefore be used by dijkstras
// Inputs:
//   obstacles:  Grown polygons consisting of the coordinates
//				 which make up their inner and outter vertices'
// Outputs:
//   obstacles:  A reduced representation of the same polygons
//				 by only the vertices of importance, i.e the
//				 covex hull of each polygon.
void algs::replaceWithConvexHulls(vector<Polygon*> &obstacles)
{
	//Emily
	//code goes here
}

// Given the vector of polygons (obstacles) and the member
// variables curr_pos and goal, determine which hulls/polygons 
// are no longer of importance and can be disregarded from this
// point in the journey forward
// Inputs:
//   obstacles:  Covex hulls of all obstacles in the course
// Outputs:
//   obstacles:  A recuded vector of Polygons containing only 
//				 those obstacles yet to be passed
void algs::removeHullsPassed(vector<Polygon*> &obstacles)
{
	//Kira
	//code goes here
}

bool sortByAngles(std::pair<coord, std::pair<double, double>> pair1, std::pair<coord, std::pair<double,double>> pair2)
{
    //if angles are the same, evaluate by distance
    if(pair1->second->first == pair2->second->first)
        return pair1->second->second < pair2->second->second;
    
    //angles are not the same, use them for sorting comparison
    return pair1->second->first < pair2->second->first;
}
bool sortByDistances(std::pair<std::pair<coord, coord>, double> pair1, std::pair<std::pair<coord, coord>, double> pair2)
{
    return pair1->second < pair2->second;
}

/*
 Input: a set of disjoint polygonal obstacles
 Output: the visibility graph -- map from each vertex to all vertices visible from that vertex
 */
map<coord, vector<coord>> algs::visibilityGraph(const vector<Polygon *> &obstacles)
{
    map<coord, vector<coord>> visibility_graph;
    vector<Polygon *>::iterator iter_obstacles;
    for(iter_obstacles = obstacles.begin(); iter_obstacles != obstacles.end(); ++iter_obstacles){
        
        vector<coord> vertices = (*iter_obstacles)->coord_;
        vector<coord>::iterator iter_vertices;
        for(iter_vertices = vertices.begin(); iter_vertices != vertices.end(); ++iter_vertices){
            
            visible = visibleVertices(*iter_vertices, obstacles)
            visibility_graph.push_back(std::make_pair(*iter_vertices, visible));
        }
        
    }
    
}


/*
 Input: a set of obstacles and a point
 Output: the set of obstacle vertices visible from the given point
 */
vector<coord> algs::visibleVertices(const coord &point, const vector<Polygon*> &obstacles)
{
    
    vector<Polygon *>::iterator iter_obstacles;
    vector<coord>::iterator iter_vertices;
    vector<pair<coord, double>>::iter_angles;
    
    
    
    //create map with keys = obstacle vertices, values = clockwise angle that the half-line from point to each vertex makes with the positive x-axis
    vector<pair<coord, pair<double, double>>> vertices_angles;
    
    //find obstacle edges intersected by half-line extending from point and store in tree
    vector<<pair<pair<coord, coord>, double>> intersecting_edges;
    
    for(iter_obstacles = obstacles.start(); iter_obstacles != obstacles.end(); ++iter_obstacles){
        vertices<coord> vertices = (*iter_obstacles)->coord_;
        for(iter_vertices = vertices.start(); iter_vertices != vertices.end(); ++iter_vertices){
            //compute angle between point and vertices
            double angle = atan2(iter_vertices->xy_[1] - point.xy_[1], iter_vertices->xy_[0] - point.xy_[0]) - atan2(point.xy_[1], point.xy_[0] + 1);
            double distance = sqrt((iter_vertices->xy_[1] - point.xy_[1])^2 + (iter_vertices->xy_[0] - point.xy_[0])^2);
            vertices_angles.push_back(std::make_pair(*iter_vertices, std::make_pair<angle, distance)));
            
        }
        
        //check for half-line intersection
        for(int i = 0; i < vertices.size(); ++i){
            
            point.xy_[0]
            point.xy_[1]
            
            vertices[i].xy_[0]
            vertices[i].xy_[1]
            
            vertices[(i + 1)%vertices.size()].xy_[0]
            vertices[(i + 1)%vertices.size()].xy_[1]
            
            //push into map if edge intersects half-line
            intersecting_edges.push_back(std::make_pair(std::make_pair(vertices[i], vertices[(i + 1)%vertices.size()]), distance));
        }
    }

    //sort the obstacle vertices according to their angle. in the case of ties, vertices closer to point should come before vertices farther from point.
    std::sort(vertices_angles.begin(), vertices_angles.end(), &sortByAngles);
    
    
    //sort the intersected edges in order in which they are interesected by half-line
    std::sort(intersecting_edges.begin(), intersecting_edges.end(), &sortByDistances);
    
    //loop through sorted list of vertices, adding vertices to list of visible vertices if visible, while managing obstacle edges
    vector<coord> visible_vertices;
    for(iter_angles = vertices_angles.start(); iter_angles != vertices_angles.end(); ++iter_angles){
        if(visible(iter_angles->first, intersecting_edges)){
            visible_vertices.push_back(iter_angles->first);
            
            intersecting_edges
            intersecting_edges
        }
    }
    
    
  
}

// Using the coords present in each Polygon within obstacles, the
// curr_pos, and the goal, find the best path to the goal.
// Continuously determine what vertices are reachable from the present position
// and then perform dijkstras algorithm in order to discover the shortest path
// to the goal
// Also updates member potential_paths will all the potential paths to be shown in
// the gui representation of the visibility graph
// Inputs:
//   obstacles:  The convex hulls of all the obstacles in the course. Whose vertices
//				 are to be used as nodes, along with curr_pos and goal, in the graph
//				 composing dijkstras algorithm
// Outputs:
//   path:    	 The coordinates, in order, to be visited by the roomba in order
//				 to perform optimally during RoboRace2013
void algs::dijkstra(vector<Polygon*> &obstacles, vector<coord> &path)
{
    
    vector<coord> queue;
    determineReachableCoords(source, obstacles, queue);
    std::map<coord, bool> visited;
    
    while(!queue.isEmpty()){
        coord = queue.pop_back();
        
    }
    
    
	vector<Polygon *>::iterator iter_obstacles;
    for(iter_obstacles = obstacles.start(); iter_obstacles != obstacles.end(); ++iter_obstacles){

    }
}

// Given the curr_pos of the roomba and the path by which to follow
// determine the next appropriate step by which to move the roomba
// and update its curr_pos to that completed step
// If goal is reached, replaces curr_pos with goal
// Inputs:
//   path:  The sequence of markers the robot should strive to reach
// Outputs:
//   angle:  The number of degrees by which to turn
//   dist: 	 The distance to travel forward along the current direction
//	 path:   Updates path by removing those points which are reached/passed
void algs::pathPlan(vector<coord> &path, double &angle, double &dist)
{
	//Henrique
	//removes passed coords from path
	//replaces curr with goal on finish
	//updates curr_pos assuming succesful step taken
	//returns next differential command for roomba
	//code goes here
}

// A visual representation of the workspace of the robot, along with its potential
// and chosen paths from start to goal.
// Inputs:
//   map_id: 	The title or index of the visibility graph version. Will increase
//			 	as collisions occur and new paths have to be computed
//   path:   	The path chosen as the optimal path to be highligted in the gui
//   boundary:  The outermost edge of the workspace of the robot
//   o_obs: 	The original, ungrown obstacles in the course
//	 g_obs: 	The convex hulls of the grown obstacles in the course
// Outputs:
//   Renders an on screen GUI representation of the Visibility Graph
void algs::renderVisibilityGraph(int map_id, vector<coord> &path, Polygon &boundary, vector<Polygon*> &o_obs, vector<Polygon*> &g_obs)
{
	//?
	//code goes here
}
