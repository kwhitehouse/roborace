#ifndef POLYGON_H_
#define POLYGON_H_

#include <vector>
#include <limits>

class coord;



using namespace std;

//
// Polygon class: all obstacles should derive from this
//
class Polygon {
    
public:
    Polygon (vector<coord> &coords) {
        coords_ = coords;
        //grab bounding box
        x_max = -std::numeric_limits<double>::max();
        y_max = -std::numeric_limits<double>::max();
        x_min = std::numeric_limits<double>::max();
        y_min = std::numeric_limits<double>::max();
        for(int i = 0; i < coords_.size(); ++i ){
            if(coords_[i][0] < x_min ){
                x_min = coords_[i][0];
            }
            if(coords_[i][1] < y_min ){
                y_min = coords_[i][1];
            }
            if(coords_[i][0] > x_max ){
                x_max = coords_[i][0];
            }
            if(coords_[i][1] > y_max ){
                y_max = coords_[i][1];
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