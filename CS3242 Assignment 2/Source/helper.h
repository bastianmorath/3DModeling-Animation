//
//  helper.hpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#define MAXV 1000000
#define MAXT 1000000

#include <stdio.h>
#include <set>
#include <map>
#include <Eigen/Dense>


namespace helper {
    
    int getIndexNotYetSeen(int tcount, const std::set<int> t_v);
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2);
    std::pair<int, int> getVerticesForVersion(int triangleList[MAXT][3], const int t_triangleIndex, const int t_version);
    void findNeighbors(int fNextList[MAXT][3], std::vector<std::set<int> > &t_v, std::set<int> &t_seenIndices, const int t_index);
};
