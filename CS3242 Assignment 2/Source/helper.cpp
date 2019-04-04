//
//  helper.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#include "helper.h"
#include <utility>

#include <set>
#include <map>
#include <vector>

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

    /**
     * @desc Finds all neighbors of a triangle
     * @param vector<set<int>> &t_v - a vector containing a set of triangles for each different component
     * @param set<int> &t_seenIndices - All vertices that we have traversed so far. Used to terminate recursion
     * @param int t_index - Index of triangle that we should start our search from
     */
    void findNeighbors(int fNextList[MAXT][3], std::vector<std::set<int>> &t_v, std::set<int> &t_seenIndices, const int t_index) {
        int numComponents = int(t_v.size());
        t_seenIndices.insert(t_index);
        t_v[numComponents-1].insert(t_index);
        for (int version=0; version <3; version++) {
            int orTri_neighbor = fNextList[t_index][version];
            if (orTri_neighbor != 0) { // If no edge vertex
                int neighbor_index = orTri_neighbor >> 3;
                if (t_v[numComponents-1].find(neighbor_index) == t_v[numComponents-1].end()) { // Element not yet seen
                    helper::findNeighbors(fNextList, t_v, t_seenIndices, neighbor_index);
                }
            }
        }
        
    }
    
}
