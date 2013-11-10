#include <iostream>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include <time.h>

using namespace std;

//	hw4 team 11
//	RoboRace 2013
int main (int argc, const char * argv[])
{
    if (argc != 3) { //bad input
        cout << "usage: hw4_team11 <obstacles_file>.txt <start_goal_file>.txt " << endl;
        return -1;
    }
    time_t timer;
    time(&timer);  /* get current time */

    Polygon *boundary;
    vector< Polygon* > obstacles;
    Parser parser;
    parser.parseObstacles(argv[1], boundary, obstacles);

    coord start;
    coord goal;
    parser.parseStartGoal(argv[2], start, goal);
    vector< Polygon* > orig_obs = obstacles;

    algs code = algs(start, goal);
    code.growObstacles(obstacles);
    // code.replaceWithConvexHulls(obstacles);

    // vector<coord> path;
    // coord curr_pos = start;
    // int map_num = -1;
    // bool bump_hit = false;
    // bool visual_hit = false;
    // double angle = 0;
    // double dist = 0;

    // while(curr_pos != goal){ //assumes goal is reachable

    //     //determine path
    //     code.removeHullsPassed(obstacles);
    //     code.dijkstra(obstacles, path);
    //     code.renderVisibilityGraph(++map_num, path, boundary, orig_obs, obstacles);

    //     while(curr_pos != goal){
    //         //determine step distance or angle turn
    //         code.pathPlan(path, angle, dist); //removes passed coords // replaces curr with goal on finish
    //         //send command to roomba
    //         ????????send
    //         //wait and listen for roomba reply when finished
    //         ???????receive
    //         //////interpret reply for camera image and bump sensors
    //         ??????interpret
    //         if(bump_hit || visual_hit){
    //             break;
    //         }
    //     }
    // }

    double seconds = difftime(time(NULL),timer);
    cout << "Goal achieved at time: " << seconds << endl;
    return 0;
}


// //on robot side
// while (true){
//     while(waiting for command){
//         report status;    
//     }
//     while(command not finished){
//         perform command
//     }
//     full stop;
// }

