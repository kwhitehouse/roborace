#include "algs.h"
#include <math.h>
#include <GLUT/glut.h>

#define PI 3.14159265

// Given the initial vector of polygons (obstacles), grow each one
// individually (all the way around) by some pretermined region
// in order to determine the workspace of the robot without collisions
// Inputs:
//   obstacles:  Original polygons consisting of the coordinates
//                               which make up their vertices
// Outputs:
//   obstacles:  The same polygons with coordinates added to each.
//                       polygon accounting for their growth regions
vector<Polygon*> algs::createReflections(vector<Polygon*> &obstacles)
{

    //Growing dimensions
    double radius = sqrt(pow(0.34306,2.0)/2);

    //iterators
    vector<Polygon*>::iterator itp;
    vector<coord>::iterator itc;

    //reflected obstacles to return
    vector< Polygon* > reflected_obstacles;
    vector< coord > reflected_coords;

    //double area, centroid_x, centroid_y; 
    //iterate through all polygons and compute centroid
    for(itp = obstacles.begin(); itp != obstacles.end(); ++itp) {
        //area = centroid_x = centroid_y = 0;
        vector<coord> coords = (*itp)->coords_;
        //reflected_coords.clear();
        vector< coord > reflected_coords;

        if (coords.size() != 4){
            radius = radius*2;
        }
        for(vector<coord>::size_type i = 0; i < coords.size(); ++i) {
            int one = i-1;
            
            if( one < 0 ){
                one = (int) coords.size() - 1;
            }

            int two = i;
            int three = (i + 1)%coords.size();
            coord c1 = coords[one];
            coord c2 = coords[two];
            coord c3 = coords[three];

            double x1 = (c1.x - c2.x);
            double y1 = (c1.y - c2.y);

            double total = sqrt(pow(x1, 2.0) + pow(y1, 2.0));
            x1 = x1/total;
            y1 = y1/total;

            double x2 =  (c2.x - c3.x);
            double y2 = (c2.y - c3.y);
            total = sqrt(pow(x2, 2.0) + pow(y2, 2.0));
            x2 = x2/total;
            y2 = y2/total;

            double x3 =  (x2 - x1);
            double y3 = (y2 - y1);
            total = sqrt(pow(x3, 2.0) + pow(y3, 2.0));
            x3 = x3/total;
            y3 = y3/total;

            cout << "Current Coord: "<< "("<< c2.x << ","<<c2.y<<")"<<endl;
            cout << "("<<x3 << ","<<y3<<")"<<endl;

            // double diff = abs(x3) - abs(y3);
            // if (abs(diff) < 0.1) {
            //     if (x3 < 0) x3 = -1;
            //     else if(x3 > 0) x3 = 1;
            //     if(y3 < 0) y3 = -1;
            //     else if (y3 > 0) y3 = 1;
            // }
            // else if(diff > 0) {
            //     if (x3 < 0) x3 = -2;
            //     else if(x3 > 0) x3 = 2;
            //     if(y3 < 0) y3 = -1;
            //     else if (y3 > 0) y3 = 1;
            // }
            // else {
            //     if (x3 < 0) x3 = -1;
            //     else if(x3 > 0) x3 = 1;
            //     if(y3 < 0) y3 = -2;
            //     else if (y3 > 0) y3 = 2;

            // } 
        

            reflected_coords.push_back(coord(c2.x + radius*x3, c2.y + radius*y3));
        }

        for(vector<coord>::size_type i = 0; i < reflected_coords.size(); ++i) {
            // int one = i-1;
            
            // if( one < 0 ){
            //     one = (int) coords.size() - 1;
            // }

            int one = i;
            int two = (i + 1)%coords.size();
            int three = (i + 1 + coords.size())%coords.size();
            coord c1 = coords[one];
            coord c2 = coords[two];
            coord c3 = coords[three];
            coord &r1 = reflected_coords[one];
            coord &r2 = reflected_coords[two];
            //coord &r3

            if(c1.x == c2.x){
                cout << "EQUAL X" << endl;

                if( abs(r1.x) <  abs(r2.x) ){
                    //r1.x = 1.3*r1.x;
                    r2.x = r1.x;
                }
                else{
                    //r2.x =1.3*r2.x;
                    r1.x = r2.x;
                }

            }

             if(c1.y == c2.y){
                cout << "EQL Y" << endl;
                if( abs(r1.y) <  abs(r2.y) ){
                    //r1.y = 1.3*r1.y;
                    r2.y = r1.y;
                }
                else{
                    //r2.y = r2.y*1.3;
                    r1.y = r2.y; 
                }

            }

        }
        //max of reflected points...
        ///okay.


        reflected_obstacles.push_back(new Polygon(reflected_coords));
    }
    return reflected_obstacles;
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

bool sortByAngle(std::pair<coord, pair<float, float> > pair1, std::pair<coord, pair< float, float> > pair2)
{
    if (pair1.second.first == pair2.second.first)
        return pair1.second.second <= pair2.second.second;
    return pair1.second.first < pair2.second.first;
}

bool strictlyLeft(const coord &a, const coord &b, const coord &c)
{
    return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) < 0;
}

vector<Polygon *> algs::createConvexHulls(const vector<Polygon*> &obstacles)
{
    //Create vector of polygon* to save convex hull of obstacles
    vector<Polygon*> hulls;

    /*find rightmost, lowest point*/
    std::vector<Polygon*>::const_iterator itp;
    for(itp = obstacles.begin(); itp != obstacles.end(); ++itp) {
        vector<coord> coords = (*itp)->coords_;
        coord rl = coords.front();
        std::vector<coord>::iterator itc;
        for(itc = coords.begin(); itc != coords.end(); ++itc) {
            if(itc->x >= rl.x && itc->y <= rl.y)             
                rl = *itc;
        }


        /*sort angles around rightmost lowest point*/
        std::vector<pair<coord, pair<float, float> > > angles;
        for(itc = coords.begin(); itc != coords.end(); ++itc) {
            float angle = atan2(itc->y - rl.y, itc->x - rl.x);
            float distance = sqrt(pow(itc->x - rl.x, 2.0) + pow(itc->y - rl.y, 2.0));
            angles.push_back(make_pair(*itc, make_pair(angle, distance))); 
        }
        sort(angles.begin(), angles.end(), sortByAngle);

        /*push last and first sorted angle points onto stack*/
        vector<coord> stack;
        stack.push_back(angles.back().first);
        stack.push_back(angles.front().first);

        /*if point is strictly left push onto stack and increment, else pop stack*/ 
        vector<coord>::size_type i = 1;
        while(i < angles.size() - 1){
            if(strictlyLeft(angles[i].first, stack.back(), stack[stack.size() - 2])){
                stack.push_back(angles[i].first);
                ++i;
            }
            else 
                stack.pop_back();
        } 

        cout << "hull size: " << stack.size() << endl;
        hulls.push_back(new Polygon(stack));
    }
    return hulls;

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

   //cout << "visible vertices size: " << visible.size() << endl;
   //cout << "edges size: " << edges.size() << endl;

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

bool pointWithinPolygon(vector<coord> polygon, const coord &point)
{
    vector<Polygon *>::size_type i, j;
    bool inside = false;

    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if (((polygon[i].y >= point.y) != (polygon[j].y >= point.y)) &&
                (point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
            inside = !inside;
    }
    return inside;
}


map<coord, vector<coord> > algs::constructVisibilityGraph(const vector<Polygon *> &obstacles, const Polygon* &boundary, const coord &start, const coord &goal)
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
        vector<coord> coords_b = boundary->coords_;
        for(vector<coord>::size_type j = 0; j < coords_b.size(); ++j) {
            edges.push_back(make_pair(coords_b[j], coords_b[(j + 1)%coords_b.size()]));    
        }


    //check which points are within other obstacles
    bool hidden;
    map<coord, bool> hidden_vertices;
    for(vector<Polygon *>::size_type i = 0; i < obstacles.size(); ++ i) {
        vector<coord> coords = obstacles[i]->coords_;
        for(vector<coord>::size_type k = 0; k < coords.size(); ++k) {
            hidden = false;
            for(vector<Polygon *>::size_type j = 0; j < obstacles.size(); ++ j) {
                if(i != j && pointWithinPolygon(obstacles[j]->coords_, coords[k]))
                        hidden = true;
            }
            hidden_vertices[coords[k]] = hidden;
        }
    }
    
    vector<coord> vertices;
    //iterate through obstacles, draw lines to each vertex and check if they intersect obstacle edges
    for(vector<Polygon *>::size_type i = 0; i < obstacles.size(); ++ i) {
        vector<coord> coords = obstacles[i]->coords_;
        for(vector<coord>::size_type j = 0; j < coords.size(); ++j) {
            
            vertices.clear();
            
            if(!hidden_vertices[coords[j]]){
                for(vector<Polygon *>::size_type m = 0; m < obstacles.size(); ++m) {
                    vector<coord> obs_coords = obstacles[m]->coords_;
                    if(i != m){
                        for(vector<coord>::size_type n = 0; n < obs_coords.size(); ++n) {
                                if(!hidden_vertices[obs_coords[n]])
                                        vertices.push_back(obs_coords[n]);
                        }
                    }
                }

                vertices = visibleVertices(edges, vertices, coords[j]);

                vertices.push_back(coords[(j + 1)%coords.size()]);
                vertices.push_back(coords[(j - 1 + coords.size())%coords.size()]);
            }

            cout << "checking vertex: " << coords[j].x << " ," << coords[j].y << endl;
            cout << "visible number : " << vertices.size() << endl;
            visibility_graph[coords[j]] = vertices;
        }
    }

    vertices.clear();
    for(vector<Polygon *>::size_type m = 0; m < obstacles.size(); ++m) {
        vector<coord> coords = obstacles[m]->coords_;
        for(vector<coord>::size_type n = 0; n < coords.size(); ++n) {
                if(!hidden_vertices[coords[n]])
                        vertices.push_back(coords[n]);
        }
    }

    vector<coord> start_visible = visibleVertices(edges, vertices, start);
    visibility_graph[start] = start_visible; 
    for(vector<coord>::size_type a = 0; a < start_visible.size(); ++a)
        visibility_graph[start_visible[a]].push_back(start);

    vector<coord> end_visible = visibleVertices(edges, vertices, goal);
    visibility_graph[goal] = end_visible;
    for(vector<coord>::size_type a = 0; a < end_visible.size(); ++a)
                visibility_graph[end_visible[a]].push_back(goal);
    
    return visibility_graph;
}

bool algs::segmentsIntersect(const coord &c1, const coord &c2, const coord &c3, const coord &c4)
{
    coord s, t;
    s.x = c2.x - c1.x;
    s.y = c2.y - c1.y;
    t.x = c4.x - c3.x;
    t.y = c4.y - c3.y;

    double u, v;
    u = (-s.y * (c1.x - c3.x) + s.x * (c1.y - c3.y)) / (-t.x * s.y + s.x * t.y);
    v = ( t.x * (c1.y - c3.y) - t.y * (c1.x - c3.x)) / (-t.x * s.y + s.x * t.y);

    if (u >= 0 && u <= 1 && v >= 0 && v <= 1){
        coord intersection;
        intersection.x = c1.x + (v * s.x);
        intersection.y = c1.y + (v * s.y);
        double d1 = sqrt(pow(intersection.x - c1.x, 2.0) + pow(intersection.y - c1.y, 2.0));
        double d2 = sqrt(pow(intersection.x - c2.x, 2.0) + pow(intersection.y - c2.y, 2.0));
        double d3 = sqrt(pow(intersection.x - c3.x, 2.0) + pow(intersection.y - c3.y, 2.0));
        double d4 = sqrt(pow(intersection.x - c4.x, 2.0) + pow(intersection.y - c4.y, 2.0));
        return !((d1 < 0.01 || d2 < 0.01) && (d3 < 0.01 || d4 < 0.01));

        return !((intersection == c1 || intersection == c2) ||
               (intersection == c3 || intersection == c4));
    }
    
    return false;


/*





        if(c1.x == 1 && c1.y == 1){
                cout << "c1: " << c1.x << ", " << c1.y << " c2: " << c2.x << " , " << c2.y << endl;
                cout << "c3: " << c3.x << ", " << c3.y << " c4: " << c4.x << " , " << c4.y << endl;
        }


        double r_cross_s = c1.x*c3.y - c1.y*c3.x;
        //check if parallel or infinitely intersecting
        if(r_cross_s == 0.0)
                return false;

        coord q_minus_p = coord(c4.x - c2.x, c4.y - c2.y);
        double q_minus_p_cross_s = q_minus_p.x*c3.y - q_minus_p.y*c3.y;
        double q_minus_p_cross_r = q_minus_p.x*c1.y - q_minus_p.y*c1.y;
        double t = q_minus_p_cross_s/r_cross_s;
        double u = q_minus_p_cross_r/r_cross_s;
        //cout << "t: " << t << " u: " << u << endl;
        coord intersection = coord(c4.x + u*c3.x, c4.y + u*c3.y);
        if(c1.x == 1 && c1.y == 1)
                cout << "intersection: " << intersection.x << ", " << intersection.y << endl;
        //if((intersection == c1 || intersection == c2) && (intersection == c3 || intersection == c4))
         //       return false;
        
        return (0 <= t && t <= 1 && 0 <= u && u <= 1);




    double denominator = (c1.x - c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x - c4.x); 

    //lines are parallel, if they are the same line they are visible anyway so return false
    if (denominator == 0 )
        return false;

    //coord intersection;
    intersection.x = (c1.x*c2.y - c1.y*c2.x)*(c3.x - c4.x) - (c1.x - c2.x)*(c3.x*c4.y - c3.y*c4.x);
    intersection.x /= denominator;
    intersection.y = (c1.x*c2.y - c1.y*c2.x)*(c3.y - c4.y) - (c1.y - c2.y)*(c3.x*c4.y - c3.y*c4.x);
    intersection.y /= denominator;
   
   if(intersection == c1 || intersection == c2 || intersection == c3 || intersection == c4)
        return false;
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
*/
}

bool containsUnvisited(pair<coord, bool> vertex)
{
    return !vertex.second;
}


bool coordDistComp(const coord *lhs, const coord *rhs){
    return  lhs->dist > rhs->dist;
}

double euclidDist(const coord a, const coord b){
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)); // L2 norm
}

vector<coord*> algs::dijkstra(coord &source, map<coord, vector<coord> > &visibility_graph, coord &target){

    map<coord, coord*> visibility_reference;
    map<coord, vector<coord> >::iterator iter_g;
    for(iter_g = visibility_graph.begin(); iter_g != visibility_graph.end(); ++iter_g){
        visibility_reference[iter_g->first] = new coord(iter_g->first.x, iter_g->first.y);
    }
    visibility_reference[source] = &source;
    visibility_reference[target] = &target;

    source.dist = 0;
    vector<coord*> minHeap;
    minHeap.push_back(visibility_reference[source]);

    coord* vert;
    while (minHeap.size() > 0) {
        vert = minHeap.back();

        minHeap.pop_back();
        if(*vert == target){
            break;
        }
        vert->known = true;

        vector<coord> neighbors =  visibility_graph[*vert];
        for(int i = 0; i < (int) neighbors.size(); ++i){

            coord *neighbor = visibility_reference[ neighbors[i] ];
            double accumulate = vert->dist + euclidDist(*vert, *neighbor);

            if( !neighbor->known && accumulate < neighbor->dist ){
                neighbor->dist = accumulate;          
                neighbor->previous = vert;
                minHeap.push_back(neighbor);
                std::sort (minHeap.begin(), minHeap.end(), coordDistComp);
            }
        }
    }
    vector<coord*> path;

    while( vert != NULL){
        cout << " -curr- " << *vert << endl;

        path.push_back(vert);
        vert = vert->previous;
    }
    std::reverse(path.begin(), path.end());

    return path;
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

//Calculates distance + angle from current location to next location
vector<float> algs::getPathInfo(coord &c1, coord &c2){
    vector<float> path;

    float dist = sqrt(pow(c2.x - c1.x, 2.0) + pow(c2.y - c1.y, 2.0));
    float angle = atan2(c2.y - c1.y, c2.x - c1.x) * 180/ PI;
    path.push_back(dist);
    path.push_back(angle);

    return path;

}

