#ifndef __PARSER_H__
#define __PARSER_H__

#include <iostream>
#include <vector>

class Polygon;
class coord;

//
// Basic parser for the text files:
//
class Parser {
public:
    void parseObstacles (
            const char *file,
            Polygon* &boundary,
            std::vector< Polygon * > &obstacles);
    
    void parseStartGoal (
            const char *file,
            coord &start,
            coord &goal);
};

#endif
