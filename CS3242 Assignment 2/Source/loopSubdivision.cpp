
//
//  loopSubdivision.h
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 19.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#ifndef loopSubdivision_h
#define loopSubdivision_h

#include <stdio.h>
#include "loopSubdivision.h"
#endif /* loopSubdivision_h */

namespace loopSubdivision
{
    /**
     * @desc Determines whether two numbers are almost equal, i.e. absolut distance is smaller than machine epsilon
     * @param T x - First number
     * @param T y - Second number
     * @param int ulp - some margin
     * @return bool - Whether two numbers are almost equal
     */
    template <class T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
    {
        // the machine epsilon has to be scaled to the magnitude of the values used
        // and multiplied by the desired precision in ULPs (units in the last place)
        return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
            // unless the result is subnormal
            || std::abs(x - y) < std::numeric_limits<T>::min();
    }
    
    
    /**
     * @desc Adds a vertex to a list of vertices and returns the new index. If vertex already exists, we return this index instead
     * @param double vList[MAXV][3] - Array of vertices
     * @param int m_vcount - Number of vertices
     * @param Eigen::Vector3d v - Vertex to insert
     * @return bool - Index in vList where the new vertex is
     */
    std::pair<bool, int> addVertexToVertexList(double vList[MAXV][3], int m_vcount, Eigen::Vector3d v)
    {
        for (int i = 1; i <= m_vcount; i++)
        {
            if (almost_equal(vList[i][0], v[0], 2) && almost_equal(vList[i][1], v[1], 2) && almost_equal(vList[i][2], v[2], 2))
                //if (vList[i][0] == v[0]  &&  vList[i][1] == v[1]  && vList[i][2] == v[2])
                return std::make_pair(false, i); // Vertex already stored
        }
        vList[m_vcount + 1][0] = v[0];
        vList[m_vcount + 1][1] = v[1];
        vList[m_vcount + 1][2] = v[2];
        return std::make_pair(true, m_vcount + 1);
    }
    
    
    /**
     * @desc Adds a triangle to the triangle list
     * @param int tList[MAXV][3] - Array of existing triangles
     * @param int tcount - Number of triangles
     * @param Eigen::Vector3i vIndices - Triangle to insert
     */
    void addTriangleToTriangleList(int tList[MAXV][3], int tcount, Eigen::Vector3i vIndices)
    {
        tList[tcount + 1][0] = vIndices[0];
        tList[tcount + 1][1] = vIndices[1];
        tList[tcount + 1][2] = vIndices[2];
    }

    
    /**
     * @desc Returns a new odd vertex in loop subdivision when we do not have an edge-face.
     * For each edge, computes v = 3/8 * (a+b) + 1/8 * (c+d)
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param int edgeV1- Index of vertex that is on one end of the edge
     * @param int edgeV2 - Index of vertex that is on the other end of the edge
     * @param int adjV1 - Index of vertex that is opposite of the edge
     * @param int adjV2 - Index of vertex that is opposite of the edge
     * @return Eigen::Vector3d - new Vertex v = 3/8 * (a+b) + 1/8 * (c+d)
     */
    
    
    Eigen::Vector3d getOddLoopVertex(double vList[MAXV][3], int edgeV1, int edgeV2, int adjV1, int adjV2)
    {
        Eigen::Vector3d edgeVertex1(vList[edgeV1][0], vList[edgeV1][1], vList[edgeV1][2]);
        Eigen::Vector3d edgeVertex2(vList[edgeV2][0], vList[edgeV2][1], vList[edgeV2][2]);

        Eigen::Vector3d adjVertex1(vList[adjV1][0], vList[adjV1][1], vList[adjV1][2]);
        Eigen::Vector3d adjVertex2(vList[adjV2][0], vList[adjV2][1], vList[adjV2][2]);

        return 3.0 / 8.0 * (edgeVertex1 + edgeVertex2) + 1.0 / 8.0 * (adjVertex1 + adjVertex2);
    }

    
    /**
     * @desc Returns a new odd vertex in loop subdivision when we have an edge-face. For each edge, computes v = (a+b) / 2
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param int edgeV1- Index of vertex that is on one end of the edge
     * @param int edgeV2 - Index of vertex that is on the other end of the edge
     * @return Eigen::Vector3d - new Vertex v = (a+b) / 2
     */
    Eigen::Vector3d getOddLoopVertexEdge(double vList[MAXV][3], int v1Idx, int v2Idx)
    {
        Eigen::Vector3d edgeVertex1(vList[v1Idx][0], vList[v1Idx][1], vList[v1Idx][2]);
        Eigen::Vector3d edgeVertex2(vList[v2Idx][0], vList[v2Idx][1], vList[v2Idx][2]);

        return (edgeVertex1 + edgeVertex2) / 2.0;
    }

    
    /**
     * @desc Returns a new even vertex in loop subdivision when we do not have an edge-face.
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param int originalVertex - Original vertex
     * @param std::set<int> neighboringVerticesIndices - Set of all neighboring vertices
     * @param int beta_version - Version 1:  beta = 3.0 / 8 / n;
     *                           Version 2:  beta = 1. / n * (5. / 8 - pow(3. / 8 + 1. / 4 * cos(2. * M_PI / n), 2));
     * @return Eigen::Vector3d - new Vertex
     */
    Eigen::Vector3d getEvenLoopVertex(double vList[MAXV][3], int originalVertex, std::set<int> neighboringVerticesIndices, int beta_version)
    {
        Eigen::Vector3d origVertex(vList[originalVertex][0], vList[originalVertex][1], vList[originalVertex][2]);
        double n = neighboringVerticesIndices.size();

        double beta = 0;
        if (n == 3.0)
        {
            beta = 3. / 16;
        }
        else
        {
            if (beta_version == 1)
            {
                beta = 3.0 / 8 / n;
            }
            else
            {
                beta = 1. / n * (5. / 8 - pow(3. / 8 + 1. / 4 * cos(2. * M_PI / n), 2));
            }
        }
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        for (auto &vecIdx : neighboringVerticesIndices)
        {
            Eigen::Vector3d v(vList[vecIdx][0], vList[vecIdx][1], vList[vecIdx][2]);
            sum += v;
        }
        return origVertex * (1 - n * beta) + sum * beta;
    }

    
    /**
     * @desc Returns a new even vertex in loop subdivision when we have an edge-face.
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param int originalVertex - Original vertex
     * @param int v1Idx - First edge vertex index
     * @param int v2Idx - Second edge vertex index
     * @return Eigen::Vector3d - new Vertex v = 3.0 / 4 * vOrig + 1. / 8 * (v1 + v2);
     */
    Eigen::Vector3d getEvenLoopVertexEdge(double vList[MAXV][3], int originalVertex, int v1Idx, int v2Idx)
    {

        Eigen::Vector3d vOrig(vList[originalVertex][0], vList[originalVertex][1], vList[originalVertex][2]);
        Eigen::Vector3d v1(vList[v1Idx][0], vList[v1Idx][1], vList[v1Idx][2]);
        Eigen::Vector3d v2(vList[v2Idx][0], vList[v2Idx][1], vList[v2Idx][2]);

        return 3.0 / 4 * vOrig + 1. / 8 * (v1 + v2);
    }

} // namespace loopSubdivision
