#pragma once

// maximum number of vertices and triangles
#define MAXV 1000000
#define MAXT 1000000
#include <Eigen/Dense>
#include <set>

class myObjType {
public:
    myObjType() { vcount = 0; tcount = 0; };
    void draw(const bool t_smooth, const bool t_edges);
    void readFile(const char* filename);  // assumming file contains a manifold
    void writeFile(const char* filename);
    void readFilePolygon(const char* filename);

    
private:
	int vcount = 0;
	int tcount = 0;
	int triangleList[MAXT][3];      // triangle list
	int fNextList[MAXT][3];     // fnext list . Stores 0 if edge has no adjacent faces
    double vList[MAXV][3];   // vertices list
    double vertexNormalList[MAXV][3];  // storing vertex normals
    double triangleNormalList[MAXT][3];   // storing triangle normals
	double lmax[3];          // the maximum coordinates of x,y,z
	double lmin[3];          // the minimum coordinates of x,y,z
    std::map<std::set<int>, std::set<int>> adjFacesToEdge;     // faces adjacent to an edge given by two vertices
    std::map<std::set<int>, std::set<int>> adjVerticesToEdge; // vertices adjacent to an edge given by two vertices

    std::map<int, std::set<int>> adjVerticesToVertex; // faces adjacent to an edge given by two vertices

	int statMinAngle[18];
	int statMaxAngle[18];
    
    int enext(const int t_orTri);
    int sym(const int t_orTri);
    int org(const int t_orTri);
    int dest(const int t_orTri);
    
    /*
     Setup Methods
    */

   
    void initAdjacencyLists();
    void computeAngleStatistics();
    void calculateFaceNormals();
    void calculateVertexNormals();
    void computeNumberOfComponents();

    /*
        Used for orienting Triangles
    */
    bool orientTriangles();
    bool conflict(const int t1Index, const int t1Version, const int t_t2Index, const int t_t2Version);
    std::pair<bool, int> checkOrientationIndex(const int t_index, std::set<int> &t_currentComponentIds, std::set<int> &t_seenIndices);

    /*
        Used for computing Face Normals
     */
    void computeFNextList();
    void changeNeighbors(std::vector<int> t_previous_indices, const int t_currentIndex, std::vector<int> t_triangle_ids);


    /*
        Catmull-Clark Subdivision
    */
   
    void subdivideLoop();

    /*
     Helper Methods
    */
    int getIndexNotYetSeen(const std::set<int> t_v);
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2);
    std::pair<int, int> getVerticesForVersion(const int t_triangleIndex, const int t_version);
    void drawEdges();
    void findNeighbors(std::vector<std::set<int> > &t_v, std::set<int> &t_seenIndices, const int t_index);



};


