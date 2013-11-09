#ifndef PARSER_H
#define PARSER_H

#include <iostream>
#include <vector>

class coord;
class Polygon;

using namespace std;

//
// Basic parser for the text files:
//
class Parser {
public:
    virtual void parseObstacles (
            const char *file,
            Polygon *boundary,
            vector< Polygon * > &obstacles);
    
    virtual void parseStartGoal (
            const char *file,
            coord &start,
            coord &goal);
};

#endif
