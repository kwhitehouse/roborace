/*
	Initializes a new GUI called 'roborace'
*/
// void GUI::init(){
//     //glutInit(&argc, argv);
//     // glutCreateWindow("Roborace");
//     // glutInitWindowSize(500, 500);
//     // glutDisplayFunc(display);
//     // glutMainLoop();
// }

/*
	Displays red 1x1 triangle to the screen
*/

#include <GLUT/glut.h>
#include "GUI.h"

void GUI::display(void){
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT);          // Clear the color buffer (background)
    // Draw a Red 1x1 Triangle centered at origin
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_POLYGON);              // Each set of 4 vertices form a quad
    glColor3f(1.0f, 0.0f, 0.0f); // Red

    glVertex2f( 0.5f,  0.5f);
    glVertex2f(-0.5f,  0.5f);
    glVertex2f(-0.5f,  0.6f);

    glEnd();

    glFlush();
}
