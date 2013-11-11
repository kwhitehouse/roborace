#include "algs.h"
#include <math.h>
#include <GLUT/glut.h>

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
//                               which make up their vertices
// Outputs:
//   obstacles:  The same polygons with coordinates added to each.
//                       polygon accounting for their growth regions
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
        vector<coord> cset = (*it)->coords_;
    
        int len = (int)cset.size();
    
        for(int i = 0; i < len; ++i){
                        //get x coord, y coord
                        double x = cset[i].x;
                        double y = cset[i].y;
            
                        //create new coordinate + push back onto coordSet
                        cset.push_back(coord(x + dx, y + dy));
            
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
//                               which make up their inner and outter vertices'
// Outputs:
//   obstacles:  A reduced representation of the same polygons
//                               by only the vertices of importance, i.e the
//                               covex hull of each polygon.
//
// Algorithm modified from: 
// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain

void algs::replaceWithConvexHulls(vector<Polygon*> &obstacles)
{

    
    //Create vector of polygon* to save convex hull of obstacles
        vector< Polygon* > newObstacles;
    
    std::vector<Polygon*>::iterator it;
        
    //iterate through all polygons
        for( it = obstacles.begin(); it != obstacles.end(); ++it) {
                
        //create new coordinate set which will hold hull coordinates
                vector<coord> points = (*it)->coords_;
                int n = (int)points.size();
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
                for(int i = n - 2, t = k+1; i >= n; i--){
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
//      if pqr = counterclockwise turn: double < 0
//      if pqr = clockwise turn: double > 0
//      if pqr = colinear : double = -
double algs::cross(coord &p, coord &q, coord &r){
        return (q.x - p.x)*(r.y - p.y) - (r.x - p.x)*(q.x-p.x);
}

// Given the vector of polygons (obstacles) and the member
// variables curr_pos and goal, determine which hulls/polygons 
// are no longer of importance and can be disregarded from this
// point in the journey forward
// Inputs
//   obstacles:  Covex hulls of all obstacles in the course
// Outputs:
//   obstacles:  A recuded vector of Polygons containing only 
//                               those obstacles yet to be passed
void algs::removeHullsPassed(vector<Polygon*> &obstacles)
{
        //Kira
        //code goes here
}

map<coord, vector<coord> > algs::constructVisibilityGraph(const vector<Polygon *> &obstacles)
{
    map<coord, vector<coord> > visibility_graph;

    //create vectors of vertices and edges of obstacles
    vector<pair<coord, coord> > edges;
    vector<coord> vertices;
    for(vector<Polygon *>::size_type i = 0; i < obstacles.size(); ++ i) {
        vector<coord> coords = obstacles[i]->coords_;
        for(vector<coord>::size_type j = 0; j < coords.size(); ++j) {
            edges.push_back(make_pair(coords[j], coords[(j + 1)%coords.size()]));    
            vertices.push_back(coords[j]);
        }
    }

    //iterate through obstacles, draw lines to each vertex and check if they intersect obstacle edges
    for(int i = 0; i < obstacles.size(); ++ i) {
        vector<coord> coords = obstacles[i]->coords_;
        for(int j = 0; j < coords.size(); ++j) {
            //initialize visible with all vertices
            vector<coord> visible(vertices);

            for(int k = 0; k < edges.size(); ++k){ 
                vector<coord>::iterator iter = visible.begin();
                while(iter != visible.end()){
                    //if edge intersects with anything, remove from visible coords
                    if(segmentsIntersect(coords[j], *iter, edges[k].first, edges[k].second))
                        visible.erase(iter);
                    else
                        ++iter;
                }
            }
            visibility_graph[coords[j]] = visible;
        }
    }


    return visibility_graph;
}

bool algs::segmentsIntersect(const coord &c1, const coord &c2, const coord &c3, const coord &c4)
{
    double denominator = (c1.x - c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x - c4.x); 

    //lines are parallel, if they are the same line they are visible anyway so return false
    if (denominator == 0)
        return false;

    coord intersection;
    intersection.x = (c1.x*c2.y - c1.y*c2.x)*(c3.x - c4.x) - (c1.x - c2.x)*(c3.x*c4.y - c3.y*c4.x);
    intersection.x /= denominator;
    intersection.y = (c1.x*c2.y - c1.y*c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x*c4.y - c3.y*c4.x);
    intersection.y /= denominator;

    //check if intersection is within segments (c1 and c2)
    return intersection.x <= max(c1.x, c2.x) &&
           intersection.x >= min(c1.x, c2.x) && 
           intersection.y <= max(c1.y, c2.y) && 
           intersection.y >= min(c1.y, c2.y);
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
void algs::dijkstra(map<coord, vector<coord> > &visibility_graph, const coord &source)
{
    //map<coord, std::pair<coord, double> >

    //vector<coord> visible_vertices = visibility_graph[source];
    //map<coord, bool> visited;

    /*
    vector<coord> queue;
    determineReachableCoords(source, obstacles, queue);
    std::map<coord, bool> visited;
    
    while(!queue.isEmpty()){
        coord = queue.pop_back();
        
    }
	vector<Polygon *>::iterator iter_obstacles;
    for(iter_obstacles = obstacles.start(); iter_obstacles != obstacles.end(); ++iter_obstacles){

    }
     */
    

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

