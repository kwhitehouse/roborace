#include <iostream>
#include <time.h>
#include <vector>
#include <GLUT/glut.h>

#include "coord.h"
#include "Parser.h"
#include "Polygon.h"
#include "algs.h"
#include "bisc.h"
#include "robo.h"

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
    time_t timer;
    time(&timer); 

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

    reflected_obstacles = code.createReflections(original_obstacles);
    grown_obstacles = code.createConvexHulls(reflected_obstacles);

    const Polygon * c_bound = boundary;

    visibility_graph = code.constructVisibilityGraph(grown_obstacles, c_bound, start, goal);
    path = code.dijkstra(start, visibility_graph, goal);
        cout << "done with dijkstra" << endl;

    /*render in gui*/
    /*rend(argc, argv);*/

    /* ----------- ROBOT --------------- */

    //calculate angle + distance between coordinates
    cout << "\nDistance, Angle Pairs for Path" << endl;
    coord curr_pos = start;
    vector< vector<float> > moves;
    while(curr_pos != goal){ 
        //starts at 1 to skip initial position
        for (int i = 1; i <(int)  path.size(); ++i){
            coord next_pos = *path[i];
            //Gets a vector of <distance, angle (in degrees) >
            vector<float> next_move = code.getPathInfo(curr_pos, next_pos);
            moves.push_back(next_move);
            cout << "dist: " << next_move[0] << ", angle: " << next_move[1] << endl;
            curr_pos = next_pos;
        }
    }

    /*compute: grown obstacles, convex hulls, visibility graph*/
    robo robot = robo(start, goal);
    int success = robot.init("/dev/tty.ElementSerial-ElementSe");

    for(std::vector<float>::size_type i = 0; i < moves.size(); ++i){
        robot.moveNextCoord(moves[i]);
    }
    
    /* CODE RENDER CHECK */
    rend(argc, argv);
        // code.

    robot.disconnect();
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

