
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
    //Growing dimensions
    double radius = 0.34306/2;
	double dx = radius;
	double dy = radius;
    
    //iterator for polygons
	std::vector<Polygon*>::iterator it;

    //New obstacle vector
	vector< Polygon* > newObstacles;
	
    //iterate through all polygons
	for(it = obstacles.begin(); it != obstacles.end(); ++it) {
		
        //create pointer to current polygon's vector of coordinates
        vector<coord>* cset = &(**it).coords_;
        
        //Get size of old coordSet
        int len = cset.size();
		//vector<coord> c = (**it).coords_;
        
		//iterate through each polygon coordinate
		for(int  i = 0; i < len; ++i){
			//get x coord, y coord
			double x = cset[i].get(0);
			double y = cset[i].get(1);
			//create new coordinate + push back onto coordSet
			cset.push_back(*new coord(x + dx, y + dy));
		}
		//push new coord set to new Obstacles.
		//newObstacles.push_back(new Polygon(coordSet));
	}
	//set obstacles pointer to newObstacles memory location
	//obstacles = newObstacles;
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
    
    //Create vector of polygon* to save convex hull of obstacles
	vector< Polygon* > newObstacles;
    
    std::vector<Polygon*>::iterator it;
	
    //iterate through all polygons
	for( it = obstacles.begin(); it != obstacles.end(); ++it) {
		
        //create new coordinate set which will hold hull coordinates
		vector<coord> points = (**it).getCoordSet();
		int n = points.size();
		int k = 0;
		vector<coord> hull(2*n);
        
		//sort points lexicographically
		sort(points.begin(), points.end());
        
		//build lower hull
		for(int i = 0; i < n; i++){
			while(k >=2 && cross( hull[k-2], hull[k-1], points[i]) <= 0) k--;
			hull[k++] = points[i];
		}
        
		//build upper hull
		for(int i = n-2, t = k+1; i >= n; i--){
			while(k >= t && cross( hull[k-2], hull[k-1], points[i]) <= 0) k--;
			hull[k++] = points[i];
		}
        
		hull.resize(k);
        
		//push new coord set to new Obstacles.
		newObstacles.push_back(new Polygon(hull));
	}
    //set obstacles pointer to newObstacles memory location
	obstacles = newObstacles;
    
}

//Given input of 3 coordinates, calculate z-component of 3-d
//cross product.
//Inputs:
//Output:
//	if pqr = counterclockwise turn: double < 0
//	if pqr = clockwise turn: double > 0
//	if pqr = colinear : double = -
double algs::cross(coord &p, coord &q, coord &r){
    
	return (q.get(0) - p.get(0))*(r.get(1) - \
        p.get(1)) - (r.get(0) - p.get(0))*(q.get(0)-p.get(0));
	
    
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

bool sortByAngles(std::pair<coord, double> pair1, std::pair<coord, double> pair2)
{
    return pair1->second < pair2->second;
}
bool sortByDistances(std::pair<edge, double> pair1, std::pair<edge, double> pair2)
{
    return pair1->second < pair2->second;
}

/*
 Input: a set of obstacles and a point
 Output: the set of obstacle vertices visible from the given point
 */
map<coord, vector<coord>> algs::visibleVertices(const coord &point, const vector<Polygon*> &obstacles)
{
    
    vector<Polygon *>::iterator iter_obstacles;
    vector<coord>::iterator iter_obstacles;
    vertices<edge>::iterator iter_edges;
    vector<pair<coord, double>>::iter_angles;
    
    //create map with keys = obstacle vertices, values = clockwise angle that the half-line from point to each vertex makes with the positive x-axis
    vector<pair<coord, double>> vertices_angles;
    for(iter_obstacles = obstacles.start(); iter_obstacles != obstacles.end(); ++iter_obstacles){
        vertices<coord> vertices = (*iter_obstacles)->coord_;
        for(iter_vertices = vertices.start(); iter_vertices != vertices.end(); ++iter_vertices){
            //compute angle between point and vertices
            double angle = atan2(iter_vertices->xy_[1] - point.xy[1], iter_vertices->xy_[0] - point.xy[0]) - atan2(point.xy_[1], point.xy_[0] + 1);
            vertices_angles.push_back(std::make_pair(*iter_vertices, angle));
        }
    }
    
    //sort the obstacle vertices according to their angle. in the case of ties, vertices closer to point should come before vertices farther from point.
    std::sort(vertices_angles.begin(), vertices_angles.end(), &sortByAngles);
    
    //find obstacle edges intersected by half-line extending from point and store in tree
    vector<<pair<edge, double>> intersecting_edges;
    for(iter_obstacles = obstacles.start(); iter_obstacles != obstacles.end(); ++iter_obstacles){
        vertices<edge> edges = (*iter_obstacles)->edges_;
        for(iter_edges = edges.begin(); iter_edges != edges.end() ++ iter_edges){
            if(iter_edges->intersectsPointPositiveHalfLine(point)){
                double distance = iter_edges->distanceFromPoint(point);
                intersecting_edges.push_back(std::make_pair(*iter_edges, distance));
            }
        }
    }
    
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
