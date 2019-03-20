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
#include <vector>
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
    computeFNextList();
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



int myObjType::enext(int orTri)
{
    int version = orTri & 0x0000111b;
    int triangleIndex = orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 1 },
        { 1, 2 },
        { 2, 0 },
        { 3, 5 },
        { 4, 3 },
        { 5, 4 }
    };
    
    return (triangleIndex << 3) | my_map[version];
}

int myObjType::sym(int orTri)
{
    int version = orTri & 0x0000111b;
    int triangleIndex = orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 3 },
        { 1, 4 },
        { 2, 5 },
        { 3, 0 },
        { 4, 1 },
        { 5, 2 }
    };
    
    return (triangleIndex << 3) | my_map[version];
}

int myObjType::org(int orTri)
{
    int version = orTri & 0x0000111b;
    int triangleIndex = orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 0},
        { 1, 1 },
        { 2, 2 },
        { 3, 1 },
        { 4, 2 },
        { 5, 0 }
    };
    
    return tlist[triangleIndex][my_map[version]];
}

int myObjType::dest(int orTri)
{
    return org(sym(orTri));
}


// PRIVATE MEMBERS

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

void myObjType::computeFNextList() {
    // Create hash_map, that takes edge vertices as key, and returns the two triangles opposite of it
    map<pair<int, int>, pair<int, int>> mymap;
    // mymap.insert(make_pair(make_pair(1,2), 3)); //edited
    
    for (int i=1; i <= tcount; i++) {

        int v[3] = {tlist[i][0], tlist[i][1], tlist[i][2]};
        std::vector<int> v_vec (v, v+3);
        std::sort(std::begin(v_vec), std::end(v_vec));
        
        int v0 = v_vec[0];
        int v1 = v_vec[1];
        int v2 = v_vec[2];

        int f0 =  i >> 3 | distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][0]));
        int f1 =  i >> 3 | distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][1]));
        int f2 =  i >> 3 | distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][2]));
        
        // f0
        std::pair<int, int> key0 = make_pair(v0, v1);
        if (mymap.count(key0) != 0) { // One face already stored
            mymap[key0] = make_pair(mymap[key0].first, f0); //store old and new face
        } else {
            mymap.insert(make_pair(key0, make_pair(f0, 0))); //store new face, leave the other blank, i.e. 0
        }
        
        // f1
        std::pair<int, int> key1 = make_pair(v1, v2);
        if (mymap.count(key1) != 0) { // One face already stored
            mymap[key1] = make_pair(mymap[key1].first, f1); //store old and new face
        } else {
            mymap.insert(make_pair(key1, make_pair(f1, 0))); //store new face, leave the other blank, i.e. 0
        }
        
        // f2
        std::pair<int, int> key2 = make_pair(v0, v2);
        if (mymap.count(key2) != 0) { // One face already stored
            mymap[key2] = make_pair(mymap[key2].first, f2); //store old and new face
        } else {
            mymap.insert(make_pair(key2, make_pair(f2, 0))); //store new face, leave the other blank, i.e. 0
        }
    } // Hashmap created
    
    for(auto& elem : mymap)
    {
        std::cout << "{" << elem.first.first << ",  " << elem.first.second << "}: {" <<  elem.second.first << "," << elem.second.second << "}\n";
    }
    
    for (int i=1; i <= tcount; i++) {
        int o = tlist[i][0];
        int d = tlist[i][1];
        
        int current_face =  i >> 3 | 0;
      
        std::pair<int, int> opposite_faces = mymap[make_pair(o, d)];
        fnlist[i][0] = current_face = opposite_faces.first ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
        
        o = tlist[i][1];
        d = tlist[i][2];
        
        current_face =  i >> 3 | 1;
        opposite_faces = mymap[make_pair(o, d)];
        fnlist[i][1] = current_face = opposite_faces.first ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
       
        o = tlist[i][2];
        d = tlist[i][0];
        
        current_face =  i >> 3 | 2;
        opposite_faces = mymap[make_pair(o, d)];
        fnlist[i][2] = current_face = opposite_faces.first ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
    }
    
    
}

void myObjType::getVersionNumber() {
    
}
