#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include "math.h"
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "mesh.h"
#include <map>
#include <queue>
#include <iomanip>
#include <sstream>
#include <algorithm>

using namespace std;



void myObjType::draw() {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
	for (int i = 1; i <= tcount; i++)
	{
		glBegin(GL_POLYGON);
// uncomment the following after you computed the normals
	glNormal3dv(nlist[i]);
		for (int j = 0; j < 3; j++)
			glVertex3dv(vlist[tlist[i][j]]);
		glEnd();
	
	}
	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void myObjType::writeFile(char* filename)
{
    ostringstream lines;
    for (int i=1; i <= vcount; i++) {
        lines << "v " << std::to_string(vlist[i][0]) << " " << std::to_string(vlist[i][1]) << " " << std::to_string(vlist[i][2]) << endl;
    }
    for (int i=1; i <= tcount; i++) {
        lines << "f " << std::to_string(tlist[i][0]) << " " << std::to_string(tlist[i][1]) << " " << std::to_string(tlist[i][2]) << endl;
    }
    
    std::ofstream outfile (filename);
    outfile << lines.str() << std::endl;

    outfile.close();
    
}

void myObjType::readFile(char* filename)
{
	cout << "Opening " << filename << endl;
	ifstream inFile;
	inFile.open(filename);
	if (!inFile.is_open()) {
		cout << "We cannot find your file " << filename << endl;
		exit(1);
	}

	string line;
	int i, j;
	bool firstVertex = 1;
	double currCood;

	while (getline(inFile, line))
	{
		if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ')
		{
			if (line[0] == 'v')
			{
				vcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ') j++;
					currCood = vlist[vcount][k] = atof(line.substr(i, j - i).c_str());
					if (firstVertex) 
						lmin[k] = lmax[k] = currCood;
					else {
						if (lmin[k] > currCood)
							lmin[k] = currCood;
						if (lmax[k] < currCood)
							lmax[k] = currCood;
					}
					i = j;
				}

				firstVertex = 0;
			}
			if (line[0] == 'f')
			{
				tcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) {
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ' && linec[j] != '\\') j++;
					tlist[tcount][k] = atof(line.substr(i, j - i).c_str());
					i = j;
					fnlist[tcount][k] = 0;
					while (linec[j] != ' ') j++;

				}

			}


		}
	}
    calculateNormals();
    calculateAngleStatistics();
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeStat();
}


double myObjType::calculateAngle(double *a, double *b){
    double dot = dotProduct(a, b);
    
    double norm1 = sqrt(dotProduct(a, a));
    double norm2 = sqrt(dotProduct(b, b));

    return acos(dot / norm1 / norm2) *180 / M_PI;
}

void myObjType::calculateAngleStatistics()
{
    for (int i=1; i <= tcount; i++) {
        double *a = vlist[tlist[i][0]];
        double *b = vlist[tlist[i][1]];
        double *c = vlist[tlist[i][2]];
        
        double ab1 = b[0] - a[0];
        double ab2 = b[1] - a[1];
        double ab3 = b[2] - a[2];
        double ac1 = c[0] - a[0];
        double ac2 = c[1] - a[1];
        double ac3 = c[2] - a[2];
       
        double a1[] = {ab1, ab2, ab3};
        double a2[] = {ac1, ac2, ac3};

        double angle1 = calculateAngle(a1, a2);
        
        double bc1 = c[0] - b[0];
        double bc2 = c[1] - b[1];
        double bc3 = c[2] - b[2];
        
        double a3[] = {-ab1, -ab2, -ab3};
        double a4[] = {bc1, bc2, bc3};
        
        double angle2 = calculateAngle(a3, a4);
        double angle3 = 180 - angle2 - angle1;

        int minAngle = floor(std::min(std::min(angle1, angle2), angle3) / 10);
        int maxAngle = floor(std::max(std::max(angle1, angle2), angle3) / 10);
        
        statMinAngle[minAngle] += 1;
        statMaxAngle[maxAngle] += 1;

    }
}

void myObjType::calculateNormals()
{
    // We suggest you to compute the normals here
    for (int i=1; i <= tcount; i++) {
        double *v0 = vlist[tlist[i][0]];
        double *v1 = vlist[tlist[i][1]];
        double *v2 = vlist[tlist[i][2]];
        
        
        double ab1 = v1[0] - v0[0];
        double ab2 = v1[1] - v0[1];
        double ab3 = v1[2] - v0[2];
        double ac1 = v2[0] - v0[0];
        double ac2 = v2[1] - v0[1];
        double ac3 = v2[2] - v0[2];
        double cross_P[3];
        double vec_A[] = {ab1, ab2, ab3};
        double vec_B[] = {ac1, ac2, ac3};
        crossProduct(vec_A, vec_B, cross_P);
        
        nlist[i][0] = cross_P[0];
        nlist[i][1] = cross_P[1];
        nlist[i][2] = cross_P[2];
        
        
    }
}

void myObjType::crossProduct(double vect_A[], double vect_B[], double cross_P[])

{
    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[0] * vect_B[2] - vect_A[2] * vect_B[0];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

double myObjType::dotProduct(double vect_A[], double vect_B[])
{
    int product = 0;
    // Loop for calculate cot product
    for (int i = 0; i < 3; i++)
        product = product + vect_A[i] * vect_B[i];
    return product;
}

void myObjType::computeStat()
{
	int i;
    double minAngle = 0;
    double maxAngle = 0;

    
    cout << "Min. angle = " << minAngle << endl;
    cout << "Max. angle = " << maxAngle << endl;

	cout << "Statistics for Maximum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMaxAngle[i] << " ";
	cout << endl;
	cout << "Statistics for Minimum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMinAngle[i] << " ";
	cout << endl;
}
