#include <iostream>
#include <time.h>
#include <vector>
#include <GLUT/glut.h>

#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include "bisc.h"

#define WIDTH   300
#define HEIGHT  300

using namespace std;

coord start;
coord goal;

vector<Polygon *> original_obstacles;
vector<Polygon *> grown_obstacles;
map<coord, vector<coord> > visibility_graph;

void display()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-7, 13, -10, 10, 0, 1);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Set background color to white
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)

    vector<coord> cset; 
    vector<Polygon *>::iterator itp;
    vector<coord>::iterator itc;

    for(itp = original_obstacles.begin(); itp != original_obstacles.end(); ++itp){
        glPushMatrix();
        glBegin(GL_LINE_LOOP);
        glColor3f(1.0f, 0.0f, 0.0f);

        cset = (*itp)->coords_;
        for(itc = cset.begin(); itc != cset.end(); ++itc)
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y);

        glEnd();
        glPopMatrix();
    }

    for(itp = grown_obstacles.begin(); itp != grown_obstacles.end(); ++itp){
        glPushMatrix();
        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f, 0.0f, 1.0f); 

        cset = (*itp)->coords_;
        for(itc = cset.begin(); itc != cset.end(); ++itc)
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y);

        glEnd();
        glPopMatrix();
    }

    map<coord, vector<coord> >::iterator itv;
    for(itv = visibility_graph.begin(); itv != visibility_graph.end(); ++itv) {
        for(itc = itv->second.begin(); itc != itv->second.end(); ++itc){
            glPushMatrix();
            glBegin(GL_LINE_LOOP);
            glColor3f(1.0, 0, 1.0);

            glVertex2f((GLfloat) itv->first.x, (GLfloat) itv->first.y);
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y); 

            glEnd();
            glPopMatrix();
        } 
    }


    glFlush();
}




void rend(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("roborace");
    glutDisplayFunc(display);
    glutMainLoop();
}

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
   
    /*obtain obstacles*/ 
    Polygon *boundary;
    Parser parser;
    parser.parseObstacles(argv[1], boundary, original_obstacles);

    /*obtain start and goal points*/
    parser.parseStartGoal(argv[2], start, goal);

    /*compute: grown obstacles, convex hulls, visibility graph*/
    algs code = algs(start, goal);

    grown_obstacles = original_obstacles;
    code.growObstacles(grown_obstacles);
    
    vector< Polygon* > hull_obstacles(grown_obstacles);
    code.replaceWithConvexHulls(hull_obstacles);

    visibility_graph = code.constructVisibilityGraph(hull_obstacles);

    /*render in gui*/
    rend(argc, argv);

/*
     vector<coord> path;
     coord curr_pos = start;
     int map_num = -1;
     bool bump_hit = false;
     bool visual_hit = false;
     double angle = 0;
     double dist = 0;
    get current time
    time_t timer;
    time(&timer);  


 */

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

    //double seconds = difftime(time(NULL),timer);
    //cout << "Goal achieved at time: " << seconds << endl;
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

