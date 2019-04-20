//
//  helper.h
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#define MAXV 1000000
#define MAXT 1000000
#define EPSILON s

#include <Eigen/Dense>
#include <set>
#include <map>

namespace helper {

    int getIndexNotYetSeen(int tcount, const std::set<int> t_v); // returns a number [1, tcount] that is not in t_v
    
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2);
    
    // Given a triangle index and a version, returns the two vertex indices
    std::pair<int, int> getVerticesForVersion(int triangleList[MAXT][3], const int t_triangleIndex, const int t_version);
    
    bool objectHasEdges(int fNextList[MAXT][3], int triangleList[MAXT][3], int tcount);
    
    // Checks if both triangles have the same orienation
    bool sameOrientation(const int t_t1Index, const int t_t1Version, const int t_t2Index, const int t_t2Version, int triangleList[MAXT][3]);
    
    // Computes angles and number of vertices/triangles
    void computeStatistics(double vList[MAXV][3], int vcount, int triangleList[MAXT][3], int tcount);

    void fillVertexNormals(double vertexNormalList[MAXV][3], double triangleNormalList[MAXT][3], std::map<int, std::set<int>> adjFacesToVertex, int vcount);
    
    void fillFaceNormals(double triangleNormalList[MAXT][3], double vList[MAXV][3], int triangleList[MAXT][3], int tcount);
};


