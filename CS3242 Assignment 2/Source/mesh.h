#pragma once

// maximum number of vertices and triangles
#define MAXV 1000000
#define MAXT 1000000
#include <Eigen/Dense>
#include <set>

class myObjType {
	int vcount = 0;
	int tcount = 0;
	double vlist[MAXV][3];   // vertices list
	int tlist[MAXT][3];      // triangle list
	int fnlist[MAXT][3];     // fnext list . Stores 0 if edge has no adjacent faces
    double vnlist[MAXV][3];     // storing vertex normals
    double nlist[MAXT][3];   // storing triangle normals
	double lmax[3];          // the maximum coordinates of x,y,z
	double lmin[3];          // the minimum coordinates of x,y,z

	int statMinAngle[18]; // each bucket is  degrees has a 10 degree range from 0 to 180 degree
	int statMaxAngle[18];
    
private:
    double calculateAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
    void changeNeighbors(std::vector<int> previous_indices, int currentIndex, std::vector<int> triangle_ids);

public:
    void computeFNextList();

	myObjType() { vcount = 0; tcount = 0; };
	void readFile(char* filename);  // assumming file contains a manifold
	void writeFile(char* filename);  
	void draw(bool smooth, bool edges);
    void computeStat();
    void calculateFaceNormals();
    void calculateVertexNormals();
    //std::vector<int> mergeUnique(std::vector<int> vector1, std::vector<int> vector2);

    void calculateAngleStatistics();
    void computeNumberOfComponents();
    int enext(int orTri);
    int sym(int orTri);
    int org(int orTri);
    int dest(int orTri);
    
    int getIndexNotYetSeen(std::set<int> v);
    void findNeighbors(std::vector<std::set<int>> &v, std::set<int> &seenIndices, int index);
    bool orientTriangles();
    bool checkOrientationIndex(int index, std::set<int> &currentComponentIds, std::set<int> &seenIndices);
    std::pair<int, int> getVerticesForVersion(int triangleIndex, int version);
    bool conflict(int t1Index, int t1Version, int t2Index, int t2Version);
    void drawEdges(bool smooth);

};


