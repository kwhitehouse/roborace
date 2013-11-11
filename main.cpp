#include <iostream>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include <time.h>
#include "bisc.h"
#include <GLUT/glut.h>

using namespace std;

/*
	hw4 team 11
	RoboRace 2013
*/
int main (int argc, char * argv[])
{
    /*parse cmd line args*/   
    if (argc != 3) { 
        cout << "usage: hw4_team11 <obstacles_file>.txt <start_goal_file>.txt " << endl;
        return -1;
    }
    /* get current time */
    time_t timer;
    time(&timer);  

    Polygon *boundary;
    vector< Polygon* > obstacles;
    Parser parser;
    parser.parseObstacles(argv[1], boundary, obstacles);

    coord start;
    coord goal;
    parser.parseStartGoal(argv[2], start, goal);
    vector< Polygon* > orig_obs = obstacles;

    algs code = algs(start, goal);

    vector<Polygon *>::iterator it;
    cout << "ORIGINALS" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
        cout << **it << endl;

    code.growObstacles(obstacles);
    
    cout << "GROW OBSTACLES" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
        cout << **it << endl;

    code.replaceWithConvexHulls(obstacles);

    cout << "CONVEX HULLS" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
        cout << **it << endl;

    map<coord, vector<coord> > visibility_graph = code.naiveVisibilityGraph(obstacles);
    cout << "VISIBILITY GRAPH" << endl;
    map<coord, vector<coord> >::iterator vg_iter;
    for(vg_iter = visibility_graph.begin(); vg_iter != visibility_graph.end(); ++vg_iter) {
        cout << vg_iter->second.size() << endl;
    }

     vector<coord> path;
     coord curr_pos = start;
     int map_num = -1;
     bool bump_hit = false;
     bool visual_hit = false;
     double angle = 0;
     double dist = 0;

     //change if to while
     //while(curr_pos != goal){ //assumes goal is reachable

         //determine path
         //code.removeHullsPassed(obstacles);

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
    //}

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

