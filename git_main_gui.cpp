#include <iostream>
#include <vector>
#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include <time.h>
#include "bisc.h"
#include <GLUT/glut.h>
//#include <GUI.h>

using namespace std;
vector< Polygon* > orig_obs;
vector< Polygon* > grow_obs;
vector< Polygon* > hull_obs;
coord start;
coord goal;
void display(){

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-7, 13, -10, 10, 0, 1);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Set background color to black and opaque
    // glClear(GL_COLOR_BUFFER_BIT);          // Clear the color buffer (background)
    // // Draw a Red 1x1 Triangle centered at origin
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // glBegin(GL_POLYGON);              // Each set of 4 vertices form a quad
    // glColor3f(1.0f, 0.0f, 0.0f); // Red

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Set background color to white
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
    

    //draw start + end points
    glColor3f(1.0f,0.0f,0.0f);
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    glVertex2f((GLfloat) start.x, (GLfloat) start.y);
    glVertex2f((GLfloat) 10.56, (GLfloat) 0.03);
    //glVertex2f((GLfloat) goal.x, (GLfloat) goal.y);
    glEnd();

    //hardcoding x, y


    // Draw a Red 1x1 Square centered at origin
    // cout << grow_obs.size() <<endl;

    // vector<coord> cset = (*grow_obs[0]).coords_;
    // cout << cset[0].x << cset[0].y << endl;
    // cout << cset[1].x << cset[1].y << endl;
    // cout << cset[2].x << cset[2].y << endl;

    // //hardcode vectors
    // glBegin(GL_POLYGON); 
    // glColor3f(1.0f, 0.0f, 0.0f);    
    // //Polygon p = *orig_obs[0];
    
    // //cout << (GLfloat) cset[0].x
    // // glVertex2f((GLfloat) cset[0].x, (GLfloat) cset[0].y);
    // // glVertex2f((GLfloat) cset[1].x, (GLfloat) cset[1].y);
    // // glVertex2f((GLfloat) cset[2].x, (GLfloat) cset[2].y);
    
    // //WORKS
    // glVertex2f(1.0f, 10*0.0f);
    // glVertex2f(10.f, 5.f);
    // glVertex2f(05.f, 05.f);
    // glEnd();
    // glFlush();
    
    vector<Polygon *>::iterator it;

    //glFlush();

    

    for(it = grow_obs.begin(); it != grow_obs.end(); ++it){
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //glBegin(GL_POLYGON);              // Each set of 4 vertices form a quad
        //glBegin(GL_LINE_LOOP); 
        //glPushMatrix();
        
        glBegin(GL_LINE_LOOP);
        glColor3f(0.f, 1.0f, 0.0f); // Re
        vector<coord> cset = (*it)->coords_;
        cout << "grow polygon" << endl;
        for(int i = 0; i < (int)cset.size(); ++i){
            cout << cset[i].x;
            cout << cset[i].y << endl;
            glVertex2f((GLfloat) cset[i].x, (GLfloat) cset[i].y);
        }
        //glFlush();
        glEnd();
        //glPopMatrix();
        glFlush();
    }

    // glBegin(GL_LINE_LOOP); 
    // glColor3f(0.f, 0.0f, 1.0f);
    // glVertex2f(1.0f, 10*0.0f);
    // glVertex2f(10.f, 5.f);
    // glVertex2f(05.f, 05.f);
    // glEnd();

    // glFlush();


    for(it = orig_obs.begin(); it != orig_obs.end(); ++it){
        //glBegin(GL_LINE_LOOP); 
        //glPushMatrix();
        glBegin(GL_LINE_LOOP);
        glColor3f(1.0f, 0.0f, 0.0f); // Red
        vector<coord> cset = (*it)->coords_;
        cout << "new polygon" << endl;
        for(int i = 0; i < (int)cset.size(); ++i){
            cout << cset[i].x;
            cout << cset[i].y << endl;
            glVertex2f((GLfloat) cset[i].x, (GLfloat) cset[i].y);
        }
        //glFlush();
        glEnd();
        glFlush();
        //glPopMatrix();
       
       
       } //glEnd();
    }


//	hw4 team 11
//	RoboRace 2013
int main (int argc, char * argv[])
{

    // glutInit(&argc, argv);
    // glutCreateWindow("Roborace");
    // glutInitWindowSize(320, 320);
    // glutDisplayFunc(display(, 1));
    // glutMainLoop();
    // //return 0;
    




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

    // coord start;
    // coord goal;
    parser.parseStartGoal(argv[2], start, goal);
    
    //perhaps redundant..
    orig_obs = obstacles;

    algs code = algs(start, goal);

    vector<Polygon *>::iterator it;
    cout << "ORIGINALS" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it){
        cout << **it << endl;
    }



    grow_obs = code.growObstacles(obstacles);
    
    cout << "GROW OBSTACLES" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
        cout << **it << endl;

    code.replaceWithConvexHulls(obstacles);

    glutInit(&argc, argv);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Roborace");
    
    glutDisplayFunc(display);
    //this is when it starts displaying
    glutMainLoop();
    //return 0;


    cout << "CONVEX HULLS" << endl;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
        cout << **it << endl;

    //map<coord, vector<coord> > visibility_graph = code.visibilityGraph(obstacles);

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

