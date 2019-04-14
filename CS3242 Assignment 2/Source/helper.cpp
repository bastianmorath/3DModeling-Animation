//
//  helper.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//
#include <iostream>
#include "helper.h"
#include <utility>

#include <set>
#include <map>
#include <vector>

using namespace std;

namespace helper{
    
    /**
     * @desc Finds a triangle index (from 1 to tcount) that is not in t_v. Returns the smallest one
     * @param set<int> t_v - Set of indices that have already been seen/traversed
     * @return int - Smallest such index
     */
    int getIndexNotYetSeen(int tcount, const std::set<int> t_v) {
        for (int i=1; i<= tcount; i++){
            if (t_v.find(i) == t_v.end()){
                return i;
            }
        }
        return -1;
    }
    /**
     * @desc Calculates the angle between two vectors
     * @param Vector3d t_tv1 - First vector
     * @param Vector3d t_tv2 - Second vector
     * @return double - Angle in Degrees
     */
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2){
        
        double dot = t_v1.dot(t_v2);
        
        return acos(dot / t_v1.norm() / t_v2.norm()) *180 / M_PI;
    }

    /**
     * @desc Returns the two first vertices of a given triangle and version
     * @param const int t_triangleIndex - triangle index
     * @param const int t_version - version
     * @return pair<int, int> - the two first vertices of triangle with index t_triangleIndex and version t_version
     */
    std::pair<int, int> getVerticesForVersion(int triangleList[MAXT][3], const int t_triangleIndex, const int t_version) {
        int v0, v1;
        if (t_version == 0) {
            v0 = triangleList[t_triangleIndex][0];
            v1 = triangleList[t_triangleIndex][1];
            
        } else if (t_version == 1){
            v0 = triangleList[t_triangleIndex][1];
            v1 = triangleList[t_triangleIndex][2];
            
        } else {
            v0 = triangleList[t_triangleIndex][2];
            v1 = triangleList[t_triangleIndex][0];
            
        }
        return std::make_pair(v0, v1);
    }

 
    std::pair<bool, int> addVertexToVertexList(double vList[MAXV][3], int vcount, Eigen::Vector3d v){
        
        for (int i=1; i<= vcount;i++){
            //if (std::fabs(vList[i][0]-v[0]) < EPSILON   && std::fabs(vList[i][1]-v[1]) < EPSILON && std::fabs(vList[i][2]-v[2]) < EPSILON
            if (vList[i][0] == v[0]  &&  vList[i][1] == v[1]  && vList[i][2] == v[2])
                return make_pair(false, i); // Vertex already stored
        }
        vList[vcount+1][0] = v[0];
        vList[vcount+1][1] = v[1];
        vList[vcount+1][2] = v[2];
        
        return make_pair(true, vcount+1);
    }
    
    void addTriangleToTriangleList(int tList[MAXV][3], int tcount,  Eigen::Vector3i vIndices){
        tList[tcount+1][0] = vIndices[0];
        tList[tcount+1][1] = vIndices[1];
        tList[tcount+1][2] = vIndices[2];
    }
    
    /*
      For each edge, compute v = 3/8 * (a+b) + 1/8 * (c+d)
    */
    Eigen::Vector3d getOddLoopVertex(double vList[MAXV][3], int edgeV1, int edgeV2, int adjV1, int adjV2){
        Eigen::Vector3d edgeVertex1(vList[edgeV1][0], vList[edgeV1][1], vList[edgeV1][2]);
        Eigen::Vector3d edgeVertex2(vList[edgeV2][0], vList[edgeV2][1], vList[edgeV2][2]);
        
        Eigen::Vector3d adjVertex1(vList[adjV1][0], vList[adjV1][1], vList[adjV1][2]);
        Eigen::Vector3d adjVertex2(vList[adjV2][0], vList[adjV2][1], vList[adjV2][2]);
        
        return 3.0 / 8.0 * (edgeVertex1 + edgeVertex2) + 1.0 / 8.0 * (adjVertex1 + adjVertex2);
    }
    
    /*
     For each edge, compute v = (a+b) / 2
     */
    Eigen::Vector3d getOddLoopVertexEdge(double vList[MAXV][3], int v1Idx, int v2Idx){
        Eigen::Vector3d edgeVertex1(vList[v1Idx][0], vList[v1Idx][1], vList[v1Idx][2]);
        Eigen::Vector3d edgeVertex2(vList[v2Idx][0], vList[v2Idx][1], vList[v2Idx][2]);
        
        return (edgeVertex1 + edgeVertex2) / 2.0 ;
    }
    
    /*
     v_new = v * (1-n * Beta)   + (Sum all neighbors) * Beta
     */
    Eigen::Vector3d getEvenLoopVertex(double vList[MAXV][3], int originalVertex, std::set<int> neighboringVerticesIndices){
        Eigen::Vector3d origVertex(vList[originalVertex][0], vList[originalVertex][1], vList[originalVertex][2]);
        double n = neighboringVerticesIndices.size();

        double beta = 0;
        if (n==3.0) {
            beta = 3. / 16;
        } else {
            beta = 3.0 / 8 / n;
            // beta = 1. / n * (5./8 - pow(3./8 + 1./4 * cos(2.*M_PI / n), 2));
        }
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        for (auto& vecIdx: neighboringVerticesIndices) {
            Eigen::Vector3d v(vList[vecIdx][0], vList[vecIdx][1], vList[vecIdx][2]);
            sum += v;
        }
        return origVertex * (1 - n * beta) + sum * beta;
    }
    
    /*
     v_new = 3/4 * v + 1/8 *v1 + v2)
     */
    Eigen::Vector3d getEvenLoopVertexEdge(double vList[MAXV][3], int originalVertex,  int v1Idx, int v2Idx){
        
        Eigen::Vector3d vOrig(vList[originalVertex][0], vList[originalVertex][1], vList[originalVertex][2]);
        Eigen::Vector3d v1(vList[v1Idx][0], vList[v1Idx][1], vList[v1Idx][2]);
        Eigen::Vector3d v2(vList[v2Idx][0], vList[v2Idx][1], vList[v2Idx][2]);

        return 3.0 / 4 * vOrig + 1./8 * (v1 + v2);
    }
}

