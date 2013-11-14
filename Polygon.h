#ifndef POLYGON_H_
#define POLYGON_H_

#include <vector>
#include <limits>
#include "coord.h"

using namespace std;

//
// Polygon class: all obstacles should derive from this
//
class Polygon {
    
public:
    Polygon (vector<coord> &coords) {
        coords_ = coords;
        //grab bounding box
        x_max = -numeric_limits<double>::max();
        y_max = -numeric_limits<double>::max();
        x_min = numeric_limits<double>::max();
        y_min = numeric_limits<double>::max();

        for(int i = 0; i < (int) coords.size(); ++i ){
            if(coords[i].x < x_min ){
                x_min = coords[i].x;
            }
            if(coords[i].y < y_min ){
                y_min = coords[i].y;
            }
            if(coords[i].x > x_max ){
                x_max = coords[i].x;
            }
            if(coords[i].y > y_max ){
                y_max = coords[i].y;
            }
        }
    }
    
    ~Polygon ();
   
    friend ostream &operator<<(ostream &os, Polygon &p)
    {
        return os << "size: " <<  p.coords_.size();
    }
 
    vector<coord> coords_;
    // vector<bid_edge> outter_path;
    double x_max;
    double x_min;
    double y_max;
    double y_min;
};

#endif
