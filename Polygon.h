#ifndef POLYGON_H_
#define POLYGON_H_

#include <vector>
#include <limits>
#include "coord.h"

// struct bid_edge //bi-directional edge
// {
//     coord _a;
//     coord _b;
// };

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

        for(int i = 0; i < coords.size(); ++i ){
            if(coords[i].get(0) < x_min ){
                x_min = coords[i].get(0);
            }
            if(coords[i].get(1) < y_min ){
                y_min = coords[i].get(1);
            }
            if(coords[i].get(0) > x_max ){
                x_max = coords[i].get(0);
            }
            if(coords[i].get(1) > y_max ){
                y_max = coords[i].get(1);
            }
        }
    }
    
    ~Polygon ();
    
    vector<coord> coords_;
    // vector<bid_edge> outter_path;
    double x_max;
    double x_min;
    double y_max;
    double y_min;
};

#endif