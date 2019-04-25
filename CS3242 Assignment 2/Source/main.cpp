//
//  main.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#include <iostream>
#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#define FALSE 0
#define TRUE 1
#endif

using std::cout;
using std::endl;

myObjType myObj;

// global variable

bool m_Smooth = FALSE;
bool m_edges = FALSE;
bool m_Highlight = FALSE;
bool m_color_components = FALSE;
int m_loop_subdivision_beta = 1;

GLfloat angle = 0;  /* in degrees */
GLfloat angle2 = 0; /* in degrees */
GLfloat zoom = 1.0;
int mouseButton = 0;
int moving, startx, starty;

#define NO_OBJECT 10;
int current_object = 0;


void setupLighting()
{
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);

	// Lights, material properties
	GLfloat ambientProperties[] = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat diffuseProperties[] = {0.8f, 0.8f, 0.8f, 1.0f};
	GLfloat specularProperties[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat lightPosition[] = {-100.0f, 100.0f, 100.0f, 1.0f};

	glClearDepth(1.0);

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientProperties);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularProperties);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

	// Default : lighting
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
}

void display(void)
{

	float mat_specular[] = {0.3f, 0.3f, 0.3f, 1.0f};
	float mat_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
	float mat_ambient_color[] = {0.8f, 0.8f, 0.2f, 1.0f};
	float mat_diffuse[] = {0.1f, 0.5f, 0.8f, 1.0f};
	float shininess = 20;
	// glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
	glRotatef(angle2, 1.0, 0.0, 0.0);
	glRotatef(angle, 0.0, 1.0, 0.0);
	glScalef(zoom, zoom, zoom);
	myObj.draw(m_Smooth, m_edges, m_color_components);
	glPopMatrix();
	glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
	char filename[256];
	switch (key)
	{
	case 'e':
	case 'E':
		m_edges = !m_edges;
		break;
	case 'c':
	case 'C':
		m_color_components = !m_color_components;
		break;
	case 'l':
	case 'L':
		myObj.subdivideLoop(m_loop_subdivision_beta);
		break;
	case '8':
		cout << "Loop subdivison beta formula was changed to 1" << endl;
		m_loop_subdivision_beta = 1;
		break;
	case '9':
		cout << "Loop subdivison beta formula was changed to 2" << endl;
		m_loop_subdivision_beta = 2;
		break;
	case 'b':
	case 'B':
		myObj.subdivideBarycentric();
		break;
	case 'p':
	case 'P':
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	case 'w':
	case 'W':
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	case 'v':
	case 'V':
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		break;
	case 's':
	case 'S':
		m_Smooth = !m_Smooth;
		break;

	case 'h':
	case 'H':
		m_Highlight = !m_Highlight;
		break;
	case 'o':
	case 'O':
        {
		cout << "Enter the filename you want to write:";
        std::cin >> filename;
		myObj.writeFile(filename);
		break;
        }
	case 'Q':
	case 'q':
		exit(0);
		break;
	case '1':
		myObj.readFile("cube.off");
		break;
	case '2':
		myObj.readFile("teddy.obj");
		break;
	case '3':
		myObj.readFile("cat.obj");
		break;
	case '4':
		myObj.readFile("smallcaseflipped.obj");
		break;
	case '5':
		myObj.readFile("2cubes_orient_edge.obj");
		break;
	case '6':
		myObj.readFile("twocubes.obj");
		break;
    case '7':
        myObj.readFile("face.obj");
        break;
    case 'r':
    case 'R':

        {
        std::string filename;
        cout << "Enter the filename you want to open:";
        std::cin >> filename;
        myObj.readFile(filename);
		break;
        }
	default:
		break;
	}

	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mouseButton = button;
		moving = 1;
		startx = x;
		starty = y;
	}
	if (state == GLUT_UP)
	{
		mouseButton = button;
		moving = 0;
	}
}

void motion(int x, int y)
{
	if (moving)
	{
		if (mouseButton == GLUT_LEFT_BUTTON)
		{
			angle = angle + (x - startx);
			angle2 = angle2 + (y - starty);
		}
		else
			zoom += ((y - starty) * 0.001);
		startx = x;
		starty = y;
		glutPostRedisplay();
	}
}

void writeObjToFile() {
    cout << "Enter the filename you want the object to write to:";
    char name [100];
    std::cin >> name;
    
    myObj.writeFile(name);
    
}

int main(int argc, char **argv)
{

	cout << "CS3242 " << endl
		 << endl;
    // std::string filename;
	//cout << "Enter the filename you want to open:";
	//cin >> filename;

    myObj.readFile("cube.off");
    
	//cout << "1-4: Draw different objects"<<endl;
	cout << "S: Toggle Smooth Shading" << endl;
	cout << "H: Toggle Highlight" << endl;
	cout << "L: Loop Subdivision. Toggle beta-formula by pressing key 8 or 9" << endl;
	cout << "C: Color Components" << endl;
	cout << "1-7: Loop through different objects" << endl;
    cout << "R: Read specific .obj or .off file" << endl;

	cout << "B: Barycentric Subdivision" << endl;
	cout << "E: Draw Edges" << endl;
	cout << "W: Draw Wireframe" << endl;
	cout << "P: Draw Polygon" << endl;
    cout << "O: Write object to file" << endl;

	cout << "V: Draw Vertices" << endl;
	cout << "Q: Quit" << endl
		 << endl;

	cout << "Left mouse click and drag: rotate the object" << endl;
	cout << "Right mouse click and drag: zooming" << endl;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(50, 50);
	glutCreateWindow("CS3241 Assignment 3");
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);

	setupLighting();
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(/* field of view in degree */ 40.0,
				   /* aspect ratio */ 1.0,
				   /* Z near */ 1.0, /* Z far */ 80.0);
	glMatrixMode(GL_MODELVIEW);
	glutMainLoop();

	return 0;
}
