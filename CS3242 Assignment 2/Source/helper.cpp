//
//  helper.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#include "helper.h"
#include <iostream>

using std::cout;
using std::endl;

namespace helper
{
    
    /**
     * @desc Finds a triangle index (from 1 to tcount) that is not in t_v. Returns the smallest one
     * @param int tcount - Number of triangles
     * @param set<int> t_v - Set of indices that have already been seen/traversed
     * @return int - Smallest index that is not in t_v
     */
    int getIndexNotYetSeen(int tcount, const std::set<int> t_v)
    {
        for (int i = 1; i <= tcount; i++)
        {
            if (t_v.find(i) == t_v.end())
            {
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
    double calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2)
    {

        double dot = t_v1.dot(t_v2);

        return acos(dot / t_v1.norm() / t_v2.norm()) * 180 / M_PI;
    }

    
    /**
     * @desc Returns the two first vertices of a given triangle and version
     * @param int triangleList[MAXT][3] - Array of existing triangles
     * @param const int t_triangleIndex - triangle index
     * @param const int t_version - version
     * @return pair<int, int> - the two first vertices of triangle with index t_triangleIndex and version t_version
     */
    std::pair<int, int> getVerticesForVersion(int triangleList[MAXT][3], const int t_triangleIndex, const int t_version)
    {
        int v0, v1;
        if (t_version == 0)
        {
            v0 = triangleList[t_triangleIndex][0];
            v1 = triangleList[t_triangleIndex][1];
        }
        else if (t_version == 1)
        {
            v0 = triangleList[t_triangleIndex][1];
            v1 = triangleList[t_triangleIndex][2];
        }
        else
        {
            v0 = triangleList[t_triangleIndex][2];
            v1 = triangleList[t_triangleIndex][0];
        }
        return std::make_pair(v0, v1);
    }

    /**
     * @desc Calculates whether exisiting object has any edges
     * @param int fNextList[MAXT][3] - FNext datastructure
     * @param int triangleList[MAXT][3] - Array of existing triangles
     * @param int tcount - Number of triangles
     * @return bool - Whether the exisitng object has any edges
     */
    bool objectHasEdges(int fNextList[MAXT][3], int triangleList[MAXT][3], int tcount)
    {
        std::set<std::pair<int, int>> edgeVerticesSet;

        for (int i = 1; i <= tcount; i++)
        {
            // Check each triangle
            for (int version = 0; version < 3; version++)
            {
                int orTri_neighbor = fNextList[i][version];
                if (orTri_neighbor == 0)
                { // Edge vertices!
                    std::pair<int, int> edgeVertices = helper::getVerticesForVersion(triangleList, i, version);
                    edgeVerticesSet.insert(std::make_pair(edgeVertices.first, edgeVertices.second));
                }
            }
        }
        return !edgeVerticesSet.empty();
    }

    /**
     * @desc Calculates whether two triangles have the same orientation
     * @param const int t_t1Index - Index of first triangle
     * @param  const int t_t1Version - Version of first triangle
     * @param const int t_t2Index - Index of second triangle
     * @param  const int t_t2Version - Version of second triangle
     * @param int triangleList[MAXT][3] - Array of existing triangles
     * @return bool -  Whether the two triangles have the same orientation
     */
    bool sameOrientation(const int t_t1Index, const int t_t1Version, const int t_t2Index, const int t_t2Version, int triangleList[MAXT][3])
    {
        std::pair<int, int> t1Vertices = getVerticesForVersion(triangleList, t_t1Index, t_t1Version);
        std::pair<int, int> t2Vertices = getVerticesForVersion(triangleList, t_t2Index, t_t2Version);
        return t1Vertices == t2Vertices;
    }

  
    /**
     * @desc Outputs the number of triangles and vertices and angles in all triangles
     *       and counts how many times they fall into each 10-degree angle bin
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param  int vcount - Number of vertices
     * @param int triangleList[MAXT][3] - Array of existing triangles
     * @param  int tcount - Number of triangles
     */
    void computeStatistics(double vList[MAXV][3], int vcount, int triangleList[MAXT][3], int tcount)
    {
        double minAngle = 360;
        double maxAngle = 0;
        
        int statMinAngle[18] = {0};
        int statMaxAngle[18] = {0};
        
        for (int i = 1; i <= tcount; i++)
        {
            Eigen::Vector3d v1(vList[triangleList[i][0]]);
            Eigen::Vector3d v2(vList[triangleList[i][1]]);
            Eigen::Vector3d v3(vList[triangleList[i][2]]);

            Eigen::Vector3d v1_to_v2(v2 - v1);
            Eigen::Vector3d v1_to_v3(v3 - v1);
            Eigen::Vector3d v2_to_v3(v3 - v2);

            double angle1 = helper::calculateAngle(v1_to_v2, v1_to_v3);

            double angle2 = helper::calculateAngle(-v1_to_v2, v2_to_v3);
            double angle3 = 180.0 - angle2 - angle1;

            double min = std::min(std::min(angle1, angle2), angle3);
            double max = std::max(std::max(angle1, angle2), angle3);

            statMinAngle[int(floor(min / 10))] += 1;
            statMaxAngle[int(floor(max / 10))] += 1;

            minAngle = minAngle < min ? minAngle : min;
            maxAngle = maxAngle > max ? maxAngle : max;
        }
        cout << endl;
        for (int i = 0; i < 50; i++)
            cout << "#";
        cout << endl;
        cout << "Statistics for Maximum Angles" << endl;
        for (int i = 0; i < 18; i++)
            cout << statMaxAngle[i] << " ";
        cout << endl;
        cout << "Statistics for Minimum Angles" << endl;
        for (int i = 0; i < 18; i++)
            cout << statMinAngle[i] << " ";
        cout << endl;

        cout << "Min. angle = " << minAngle << endl;
        cout << "Max. angle = " << maxAngle << endl;

        cout << endl;

        cout << "No. of vertices: " << vcount << endl;
        cout << "No. of triangles: " << tcount << endl;

        cout << endl;
        for (int i = 0; i < 50; i++)
            cout << "#";
        cout << endl;
    }

    
    /**
     * @desc Calculates all vertex normals, using the average of all adjacent faces, and stores it in vertexNormalList
     * @param double vertexNormalList[MAXV][3] - Empty array where vertex normals should be filled in
     * @param  double triangleNormalList[MAXT][3] - Array with the normals of all triangles
     * @param std::map<int, std::set<int>> adjFacesToVertex - Data structure that holds all adjacent faces for each vertex
     * @param  int vcount - Number of vertices
     */
    void fillVertexNormals(double vertexNormalList[MAXV][3], double triangleNormalList[MAXT][3], std::map<int, std::set<int>> adjFacesToVertex, int vcount)
    {

        for (int i = 1; i <= vcount; i++)
        {
            std::set<int> adjacent_triangle_indices = adjFacesToVertex[i];

            Eigen::Vector3d sumVector(0, 0, 0);

            for (auto const &j : adjacent_triangle_indices)
            {
                sumVector[0] += triangleNormalList[j][0];
                sumVector[1] += triangleNormalList[j][1];
                sumVector[2] += triangleNormalList[j][2];
            }
            sumVector.normalize();

            vertexNormalList[i][0] = sumVector[0];
            vertexNormalList[i][1] = sumVector[1];
            vertexNormalList[i][2] = sumVector[2];
        }
    }

    /**
     * @desc Calculates all face normals, using the cross product, and stores it in triangleNormalList
     * @param  double triangleNormalList[MAXT][3] - Array with the normals of all triangles
     * @param double vList[MAXV][3] - Array of existing vertices
     * @param int triangleList[MAXT][3] - Array of existing triangles
     * @param  int tcount - Number of triangles
     */
    void fillFaceNormals(double triangleNormalList[MAXT][3], double vList[MAXV][3], int triangleList[MAXT][3], int tcount)
    {

        // We suggest you to compute the normals here
        for (int i = 1; i <= tcount; i++)
        {
            Eigen::Vector3d v1(vList[triangleList[i][0]]);
            Eigen::Vector3d v2(vList[triangleList[i][1]]);
            Eigen::Vector3d v3(vList[triangleList[i][2]]);

            Eigen::Vector3d v1_to_v2(v2 - v1);
            Eigen::Vector3d v1_to_v3(v3 - v1);

            Eigen::Vector3d crossP = v1_to_v2.cross(v1_to_v3);
            crossP.normalize();

            triangleNormalList[i][0] = crossP[0];
            triangleNormalList[i][1] = crossP[1];
            triangleNormalList[i][2] = crossP[2];
        }
    }

} // namespace helper
