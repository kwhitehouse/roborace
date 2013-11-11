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
vector<Polygon*> algs::growObstacles(vector<Polygon*> &obstacles)
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
        newObstacles.push_back(new Polygon(cset));
                //push new coord set to new Obstacles.
                //newObstacles.push_back(new Polygon(coordSet));
        }
        //set obstacles pointer to newObstacles memory location
       return newObstacles;
     
     
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


vector<coord> algs::visibleVertices(const vector<pair<coord, coord> > &edges, vector<coord> visible, const coord &vertex)
{
    for(vector<pair<coord, coord> >::size_type k = 0; k < edges.size(); ++k){ 
        vector<coord>::iterator iter = visible.begin();
        while(iter != visible.end()){
            //if edge intersects with anything, remove from visible coords
            if(segmentsIntersect(vertex, *iter, edges[k].first, edges[k].second))
                visible.erase(iter);
            else
                ++iter;
        }
    }
    return visible;
} 

map<coord, vector<coord> > algs::constructVisibilityGraph(const vector<Polygon *> &obstacles, const coord &start, const coord &goal)
{
    map<coord, vector<coord> > visibility_graph;

    //create vectors of edges of obstacles
    vector<pair<coord, coord> > edges;
    for(vector<Polygon *>::size_type i = 0; i < obstacles.size(); ++ i) {
        vector<coord> coords = obstacles[i]->coords_;
        for(vector<coord>::size_type j = 0; j < coords.size(); ++j) {
            edges.push_back(make_pair(coords[j], coords[(j + 1)%coords.size()]));    
        }
    }

    //iterate through obstacles, draw lines to each vertex and check if they intersect obstacle edges
    for(vector<Polygon *>::size_type i = 0; i < obstacles.size(); ++ i) {

        //create vertices of all objects except for yourself
        vector<coord> vertices;
        for(vector<Polygon *>::size_type m = 0; m < obstacles.size(); ++m) {
            vector<coord> coords = obstacles[m]->coords_;
            if(i != m){
                for(vector<coord>::size_type n = 0; n < coords.size(); ++n) {
                    vertices.push_back(coords[n]);
                }
            }
        }

        //loop through vertices of current obstacle
        vector<coord> coords = obstacles[i]->coords_;
        for(int j = 0; j < (int) coords.size(); ++j) {
            visibility_graph[coords[j]] = visibleVertices(edges, vertices, coords[j]);
        }
    }

    //create vertices of all objects for start and goal
    vector<coord> vertices;
    for(vector<Polygon *>::size_type m = 0; m < obstacles.size(); ++m) {
        vector<coord> coords = obstacles[m]->coords_;
        for(vector<coord>::size_type n = 0; n < coords.size(); ++n) {
            vertices.push_back(coords[n]);
        }
    }

    visibility_graph[start] = visibleVertices(edges, vertices, start);
    visibility_graph[goal] = visibleVertices(edges, vertices, goal);

    return visibility_graph;
}

bool algs::segmentsIntersect(const coord &c1, const coord &c2, const coord &c3, const coord &c4)
{
    double denominator = (c1.x - c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x - c4.x); 

    //lines are parallel, if they are the same line they are visible anyway so return false
    if (denominator == 0 )
        return false;

    coord intersection;
    intersection.x = (c1.x*c2.y - c1.y*c2.x)*(c3.x - c4.x) - (c1.x - c2.x)*(c3.x*c4.y - c3.y*c4.x);
    intersection.x /= denominator;
    intersection.y = (c1.x*c2.y - c1.y*c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x*c4.y - c3.y*c4.x);
    intersection.y /= denominator;


    //if lines intersect at endpoint check if they intersect middle as well
    if (c1 == c3 || c1 == c4 || c2 == c3 || c2 == c4) {
        //check if intersection is within segments (c1 and c2)
        return  intersection.x < max(c1.x, c2.x) &&
            intersection.x > min(c1.x, c2.x) && 
            intersection.x < max(c3.x, c4.x) &&
            intersection.x > min(c3.x, c4.x) && 
            intersection.y < max(c1.y, c2.y) && 
            intersection.y > min(c1.y, c2.y) &&
            intersection.y < max(c3.y, c4.y) && 
            intersection.y > min(c3.y, c4.y);
        //could we just return false here?

    }

    //check if intersection is within segments (c1 and c2)
    return  intersection.x <= max(c1.x, c2.x) &&
        intersection.x >= min(c1.x, c2.x) && 
        intersection.x <= max(c3.x, c4.x) &&
        intersection.x >= min(c3.x, c4.x) && 
        intersection.y <= max(c1.y, c2.y) && 
        intersection.y >= min(c1.y, c2.y) &&
        intersection.y <= max(c3.y, c4.y) && 
        intersection.y >= min(c3.y, c4.y);

}

bool containsUnvisited(pair<coord, bool> vertex)
{
        return !vertex.second;
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
void algs::dijkstra(map<coord, vector<coord> > &visibility_graph, const coord &source, const coord &goal)
{
    map<coord, double> distances;
    map<coord, bool> visited;
    map<coord, coord> previous;


    /*initialize all vertices as unvisited*/
    map<coord, vector<coord> >::iterator iter_g;
    for(iter_g = visibility_graph.begin(); iter_g != visibility_graph.end(); ++iter_g)
        visited[iter_g->first] = false;

    /*initialize all distances from source to each visible vertex*/
    vector<coord>::iterator iter_v;
    for(iter_v = visibility_graph[source].begin(); iter_v != visibility_graph[source].end(); ++iter_v){
            double d = sqrt(pow(iter_v->x - source.x, 2.0) + pow(iter_v->y - source.y, 2.0));
            distances[*iter_v] = d;
    }

    /*mark source as visited*/
    distances[source] = 0.0;
    vector<coord> queue;
    queue.push_back(source);

    /*while queue is not empty*/
    while(!queue.empty()){
        cout << "queue: " << queue.size() << endl;

        /*find closest vertex to current point that has not yet been visited*/
        double closest_distance = INFINITY;
        vector<coord>::iterator closest_vertex;
        for(iter_v = queue.begin(); iter_v != queue.end(); ++iter_v) {
            if(!visited[*iter_v] && distances[*iter_v] < closest_distance) {
                closest_distance = distances[*iter_v];
                closest_vertex = iter_v;
            }
        }
        coord u = *closest_vertex;
        /*remove closest, unvisited vertex from queue*/
        queue.erase(closest_vertex);
        /*mark closest, unvisited vertex as visited*/
        visited[u] = true;
        
        cout << "u: " << u.x << " " << u.y << endl;
        if(u == goal)
                return;

        /*look through unvisited neighbors of closest point*/
        vector<coord> neighbors = visibility_graph[u];
        cout << "size " << neighbors.size() << endl;
        for(iter_v = neighbors.begin(); iter_v != neighbors.end(); ++iter_v) {
            coord v = *iter_v;
            cout << "v: " << v.x << " " << v.y << endl;

            /*if not visited*/
            if(!visited[v]) {
                double dist = distances[u];
                dist += sqrt(pow(u.x - v.x, 2.0) + pow(u.y - v.y, 2.0));
                cout << "dist " << dist << endl;
                cout << distances[v] << endl;
                if(dist < distances[v]) {
                    distances[v] = dist;
                    previous[v] = u;
                    queue.push_back(v);
                }
            }
        }
   } 

   /*error check*/
   map<coord, coord>::iterator it;
   for(it = previous.begin(); it != previous.end(); ++it) {
        cout << "first, second" << endl;
       cout << it->first.x << " " << it->first.y << " " << it->second.x << " " << it->second.y << endl; 
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

