
#ifndef _COORD_H_
#define _COORD_H_

#include <cassert>
#include <iostream>
#include <cmath>

using std::istream;
using std::ostream;

class coord {
    
public:
    
    coord () {} // default constructor does nothing, not even inits.
    coord (double _x, double _y) : x(_x), y(_y) {}

    bool operator<(const coord &rhs) const
    {
        return x < rhs.x || (x == rhs.x && y < rhs.y);
    }

    friend bool operator==(const coord& lhs, const coord& rhs)
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y); 
    }

    friend bool operator!=(const coord& lhs, const coord& rhs)
    {
        return !(lhs == rhs);
    }

    friend  istream &operator>>(istream &is, coord &c) {
        return is >> c.x >> c.y;
    }
    
    friend ostream &operator<<(ostream &os, coord &c) {
        return os<<"<"<< c.x <<","<< c.y <<">";
    }
        
    double x;
    double y;
};

#endif
