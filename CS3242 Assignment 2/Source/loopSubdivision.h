//
//  loopSubdivision.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 19.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#define MAXV 1000000
#define MAXT 1000000
#define EPSILON s

#include <set>
#include <Eigen/Dense>

namespace loopSubdivision
{
    // Determines whether two numbers are almost equal, i.e. absolut distance is smaller than machine epsilon
    template <class T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp);
    
    std::pair<bool, int> addVertexToVertexList(double vList[MAXV][3], int m_vcount, Eigen::Vector3d v);

    void addTriangleToTriangleList(int tList[MAXV][3], int tcount, Eigen::Vector3i vIndices);

    // The following methods compute the new vertices in the loop-subdivision scheme

    Eigen::Vector3d getOddLoopVertex(double vList[MAXV][3], int edgeV1, int edgeV2, int adjV1, int adjV2);
    Eigen::Vector3d getOddLoopVertexEdge(double vList[MAXV][3], int v1Idx, int v2Idx);

    Eigen::Vector3d getEvenLoopVertex(double vList[MAXV][3], int originalVertex, std::set<int> neighboringVerticesIndices, int beta_version);
    Eigen::Vector3d getEvenLoopVertexEdge(double vList[MAXV][3], int originalVertex, int v1Idx, int v2Idx);
    
} // namespace loopSubdivision
