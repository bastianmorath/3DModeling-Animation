//
//  helper.hpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#define MAXV 1000000
#define MAXT 1000000
#define EPSILON s
#include <stdio.h>
#include <set>
#include <map>
#include <Eigen/Dense>


namespace helper {
    
    int getIndexNotYetSeen(int tcount, const std::set<int> t_v);
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2);
    std::pair<int, int> getVerticesForVersion(int triangleList[MAXT][3], const int t_triangleIndex, const int t_version);
   
    
    std::pair<bool, int> addVertexToVertexList(double vList[MAXV][3], int vcount, Eigen::Vector3d v);
    void addTriangleToTriangleList(int tList[MAXV][3], int tcount,  Eigen::Vector3i vIndices);

    
    Eigen::Vector3d getOddLoopVertex(double vList[MAXV][3], int edgeV1, int edgeV2, int adjV1, int adjV2);
    Eigen::Vector3d getOddLoopVertexEdge(double vList[MAXV][3], int v1Idx, int v2Idx);
    
    Eigen::Vector3d getEvenLoopVertex(double vList[MAXV][3], int originalVertex, std::set<int> neighboringVerticesIndices);
    Eigen::Vector3d getEvenLoopVertexEdge(double vList[MAXV][3], int originalVertex, int v1Idx, int v2Idx);

};


