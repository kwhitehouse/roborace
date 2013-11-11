#ifndef __GUI_H__
#define __GUI_H__

//#include <iostream>
//#include <vetor>

#include <GLUT/glut.h>

//class Polygon;
//class coord;

//char fakeParam[] = "fake";
//char *fakeargv[] = { fakeParam, NULL };
//int fakeargc = 1;
    

class GUI(argc, argv){

 	glutInit(&argc, argv);
	glutCreateWindow("Roborace");
    glutInitWindowSize(500, 500);
    glutDisplayFunc(display);
    glutMainLoop();

public:
	void display(void);

};

#endif