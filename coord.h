
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
    coord (double x, double y) {
        xy_[0] = x; xy_[1] = y;
    }

    void set (double x, double y) {
        xy_[0] = x; xy_[1] = y;
    }
    
    double get(int letter){
        return xy_[letter];
    }

    double operator[] (const int i) const;
    
    friend  istream &operator>>(istream &is, coord &c) {
        return is >> c.xy_[0] >> c.xy_[1];
    }
    
    friend ostream &operator<<(ostream &os, coord &c) {
        return os<<"<"<<c.xy_[0]<<","<<c.xy_[1]<<">";
    }
        
    double xy_[2];
};

//CHANGE TO BE WITHIN SOME THRESHOLD
inline bool operator==(const coord& lhs, const coord& rhs){
    if(lhs.xy_[0] == rhs.xy_[0]
        && lhs.xy_[1] == lhs.xy_[1]){
        return true;
    }
    return false;
}

inline bool operator!=(const coord& lhs, const coord& rhs){
    return !(lhs == rhs);
}

inline double coord::operator[]
(const int i) const
{
    assert(i >= 0 && i < 2);
    return xy_[i];
}

#endif
