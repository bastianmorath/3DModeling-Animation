#pragma once

// maximum number of vertices and triangles
#define MAXV 1000000
#define MAXT 1000000


class myObjType {
	int vcount = 0;
	int tcount = 0;
	double vlist[MAXV][3];   // vertices list
	int tlist[MAXT][3];      // triangle list
	int fnlist[MAXT][3];     // fnext list 
	double nlist[MAXT][3];   // storing triangle normals
	double lmax[3];          // the maximum coordinates of x,y,z
	double lmin[3];          // the minimum coordinates of x,y,z

	int statMinAngle[18]; // each bucket is  degrees has a 10 degree range from 0 to 180 degree
	int statMaxAngle[18];
    
private:
    double dotProduct(double vect_A[], double vect_B[]);
    double calculateAngle(double *a, double *b);
    void crossProduct(double vect_A[], double vect_B[], double cross_P[]);
    void computeFNextList();
    
public:
	myObjType() { vcount = 0; tcount = 0; };
	void readFile(char* filename);  // assumming file contains a manifold
	void writeFile(char* filename);  
	void draw();  
    void computeStat();
    void calculateNormals();
    void calculateAngleStatistics();
    void getVersionNumber();
    int enext(int orTri);
    int sym(int orTri);
    int org(int orTri);
    int dest(int orTri);


};


