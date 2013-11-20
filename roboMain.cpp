#include <iostream>
#include <time.h>
#include <vector>
#include <GLUT/glut.h>

#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include "bisc.h"

#define WIDTH   600
#define HEIGHT  600
#define DISP_VG true

using namespace std;

coord start;
coord goal;

vector<Polygon *> original_obstacles;
vector<Polygon *> reflected_obstacles;
vector<Polygon *> grown_obstacles;
Polygon* boundary;
map<coord, vector<coord> > visibility_graph;
vector<coord*> path;

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

    glColor3f(1.0f,0.0f,0.0f);
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    glVertex2f((GLfloat) start.x, (GLfloat) start.y);
    glVertex2f((GLfloat) goal.x, (GLfloat) goal.y);
    glEnd();

    vector<coord> cset; 
    vector<Polygon *>::iterator itp;
    vector<coord>::iterator itc;
    
    //boundary outline
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f, 0.0f, 1.0f); 
    cset = (boundary)->coords_;
    for(itc = cset.begin(); itc != cset.end(); ++itc)
        glVertex2f((GLfloat) itc->x, (GLfloat) itc->y);

    glEnd();


    if(DISP_VG == true)
    {
        cout << "Visibility Graph" << endl;
    }
    map<coord, vector<coord> >::iterator itv;
    for(itv = visibility_graph.begin(); itv != visibility_graph.end(); ++itv) {
        if(DISP_VG == true)
        {
        cout << "Vertex: (" << itv->first.x << "," << itv->first.y << "), " << endl;
        }
        for(itc = itv->second.begin(); itc != itv->second.end(); ++itc){
            glBegin(GL_LINE_LOOP);
            glColor3f(0.0, 1.0, 1.0);
            
            if(DISP_VG == true)
            {
                
                cout << "\t(" << itc->x << "," << itc->y << ")"<< endl; 
            }
            
            glVertex2f((GLfloat) itv->first.x, (GLfloat) itv->first.y);
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y); 

            glEnd();
        } 
    }

    
   for(itp = grown_obstacles.begin(); itp != grown_obstacles.end(); ++itp){
        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f, 0.0f, 1.0f); 

        cset = (*itp)->coords_;
        for(itc = cset.begin(); itc != cset.end(); ++itc)
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y);

        glEnd();
    }


    for(itp = original_obstacles.begin(); itp != original_obstacles.end(); ++itp){
        glBegin(GL_LINE_LOOP);
        glColor3f(1.0f, 0.0f, 0.0f);

        cset = (*itp)->coords_;
        for(itc = cset.begin(); itc != cset.end(); ++itc)
            glVertex2f((GLfloat) itc->x, (GLfloat) itc->y);

        glEnd();
    }

    for(vector<coord>::size_type i = 0 ; i < path.size() -1; ++i){
        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f, 0.0f, 0.0f);

        glVertex2f((GLfloat) path[i]->x, (GLfloat) path[i]->y); 
        glVertex2f((GLfloat) path[i+1]->x, (GLfloat) path[i+1]->y); 

        glEnd();
    } 
/*
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 1.0f);
    glVertex2f(1.17153 ,2.17153);
        glEnd();
*/

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

    printf("%s\n", biscGetVersion());
   
    /*parse cmd line args*/   
    if (argc != 3) { 
        cout << "usage: hw4_team11 <obstacles_file>.txt <start_goal_file>.txt " << endl;
        return -1;
    }
   
    /*obtain obstacles*/ 
    //Polygon *boundary;
    Parser parser;
    parser.parseObstacles(argv[1], boundary, original_obstacles);

    /*obtain start and goal points*/
    parser.parseStartGoal(argv[2], start, goal);

    /*compute: grown obstacles, convex hulls, visibility graph*/
    algs code = algs(start, goal);

   // vector<Polygon *> squares; 
   // vector<coord> c1;
   // vector<coord> c2;
   // vector<coord> c3;
   // c1.push_back(coord(0, 0));
   // c1.push_back(coord(1, 0));
   // c1.push_back(coord(1, 2));
   // c1.push_back(coord(0, 2));

   // c2.push_back(coord(0, 1.8));
   // c2.push_back(coord(6, 1.8));
   // c2.push_back(coord(6, 4));
   // c2.push_back(coord(0, 4));

   // c3.push_back(coord(5, -2));
   // c3.push_back(coord(6, -2));
   // c3.push_back(coord(6, -4));
   // c3.push_back(coord(5, -4));
 
   // squares.push_back(new Polygon(c1));
   // squares.push_back(new Polygon(c2));
   // squares.push_back(new Polygon(c3));
   // original_obstacles = squares;

    reflected_obstacles = code.createReflections(original_obstacles);
    grown_obstacles = code.createConvexHulls(reflected_obstacles);

    const Polygon * c_bound = boundary;

    visibility_graph = code.constructVisibilityGraph(grown_obstacles, c_bound, start, goal);
    path = code.dijkstra(start, visibility_graph, goal);
        cout << "done with dijkstra" << endl;

    /*render in gui*/
    /*rend(argc, argv);*/

    //ROBOT MOVEMENT
    cout << "\nDistance, Angle Pairs for Path" << endl;
    coord curr_pos = start;
    while(curr_pos != goal){ 
        //starts at 1 to skip initial position
        for (int i = 1; i <(int)  path.size(); ++i){
            coord next_pos = *path[i];
            //Gets a vector of <distance, angle (in degrees) >
            vector<float> next_move = code.getPathInfo(curr_pos, next_pos);
            cout << "dist: " << next_move[0] << ", angle: " << next_move[1] << endl;
            curr_pos = next_pos;
        }
    }
    
    /* CODE RENDER CHECK */
    /*rend(argc, argv);*/
        // code.


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

