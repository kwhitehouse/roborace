
#include <iostream>
#include <fstream>
#include "Parser.h"
#include "Polygon.h"
#include "coord.h"
#include <sstream>

using namespace std;

void Parser::parseObstacles (
        const char *file,
        Polygon* &boundary,
        vector< Polygon * > &obstacles)
{

    ifstream in(file);
    char buffer[129];
    int num_obs = 0; // keep track of how many obstacles we read in
    int line = 2;
    if(in.good()){
        in.getline(buffer,128);
        buffer[in.gcount()] = 0;
        std::istringstream iss(buffer);
        iss >> num_obs;
    }

    // make sure we read in 1 obstacle at least
    if (num_obs < 1) {
        cerr << "at least one obstacle must be defined." << endl;
    }

    for(int obs = 0; obs < num_obs; ++obs){
        int num_coords = 0;
        if(in.good()){
            in.getline(buffer,128);
            buffer[in.gcount()] = 0;
            istringstream iss(buffer);
            iss >> num_coords;
            ++line;
        }
        vector<coord> coords;
        for(int i = 0; i < num_coords; ++i){
            if(in.good()){
                in.getline(buffer,128);
                buffer[in.gcount()] = 0;
                istringstream iss(buffer);

                double x, y;
                iss >> x >> y;
                coords.push_back(*new coord(x,y));
            }
            else{
                cerr << "problem reading coordinate at line " << line << endl;
            }
            ++line;
        }

        if(obs == 0){ // grab boundary
            boundary = new Polygon(coords);
        }
        else{ // grab obstacles
            obstacles.push_back(new Polygon(coords));
        }

    }
    in.close();
}

void Parser::parseStartGoal (
        const char *file,
        coord &start,
        coord &goal)
{

    ifstream in(file);
    char buffer[129];

    double x, y;
    in.getline(buffer,128);
    buffer[in.gcount()] = 0;
    istringstream iss(buffer);
    iss >> x >> y;

    start = coord(x,y);

    in.getline(buffer,128);
    buffer[in.gcount()] = 0;
//    iss(buffer);
    iss >> x >> y;
    goal = coord(x,y);

}
