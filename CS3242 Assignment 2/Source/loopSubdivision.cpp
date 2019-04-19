
//
//  loopSubdivision.h
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 19.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#ifndef loopSubdivision_hpp
#define loopSubdivision_hpp

#include <stdio.h>
#include "loopSubdivision.h"
#endif /* loopSubdivision_hpp */


namespace loopSubdivision{
    
    template<class T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
    {
        // the machine epsilon has to be scaled to the magnitude of the values used
        // and multiplied by the desired precision in ULPs (units in the last place)
        return std::abs(x-y) <= std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp
        // unless the result is subnormal
        || std::abs(x-y) < std::numeric_limits<T>::min();
    }

    std::pair<bool, int> addVertexToVertexList(double vList[MAXV][3], int m_vcount, Eigen::Vector3d v){
        
        for (int i=1; i<= m_vcount;i++){
            if (almost_equal(vList[i][0], v[0], 2) && almost_equal(vList[i][1], v[1], 2)  && almost_equal(vList[i][2], v[2], 2) )
                //if (vList[i][0] == v[0]  &&  vList[i][1] == v[1]  && vList[i][2] == v[2])
                return std::make_pair(false, i); // Vertex already stored
        }
        vList[m_vcount+1][0] = v[0];
        vList[m_vcount+1][1] = v[1];
        vList[m_vcount+1][2] = v[2];
        return std::make_pair(true, m_vcount+1);
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

    /**
     * @desc Calculates the even vertex in loop subdivision
     ...
     * @param int beta_version - Either 1 or 2, depending which beta-formula should be choosen
     */
    Eigen::Vector3d getEvenLoopVertex(double vList[MAXV][3], int originalVertex, std::set<int> neighboringVerticesIndices, int beta_version){
        Eigen::Vector3d origVertex(vList[originalVertex][0], vList[originalVertex][1], vList[originalVertex][2]);
        double n = neighboringVerticesIndices.size();
        
        double beta = 0;
        if (n==3.0) {
            beta = 3. / 16;
        } else {
            if (beta_version == 1) {
                beta = 3.0 / 8 / n;
                
            } else {
                beta = 1. / n * (5./8 - pow(3./8 + 1./4 * cos(2.*M_PI / n), 2));
                
            }
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
