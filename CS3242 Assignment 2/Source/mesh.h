//
//  helper.hpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#pragma once

#define MAXV 1000000
#define MAXT 1000000
#include <Eigen/Dense>
#include <set>
#include <map>
#include <vector>
class myObjType
{
  public:
    myObjType()
    {
        vcount = 0;
        tcount = 0;
    };
    void draw(const bool t_smooth, const bool t_edges, const bool t_color_components);
    void readFile(const char *filename); // assumming file contains a manifold
    void writeFile(const char *filename);

    /*
     Subdivision
     */
    void subdivideLoop(int beta_version);
    void subdivideBarycentric();

  private:
    int vcount = 0;                     // number of vertices
    int tcount = 0;                     // number of triangles
    int triangleList[MAXT][3];          // row i stores the three indices of the vertices of the i-th triangle
    int fNextList[MAXT][3];             // fnext list . Stores 0 if edge has no adjacent faces
    double vList[MAXV][3];              // row i stores the x, y and z coordinate of the i-th vertex
    double vertexNormalList[MAXV][3];   // storing vertex normals
    double triangleNormalList[MAXT][3]; // storing triangle normals

    std::vector< std::vector<double> > colors; // Holds colors for each component
    double lmax[3]; // the maximum coordinates of x,y,z
    double lmin[3]; // the minimum coordinates of x,y,z

    int numUniqueComponents;                // used for computing and coloring the different components
    bool subdivided = false;                // if we subdivided, we have to recompute colors etc.
    bool edgesDrawnAfterSubdivision = true; // If we subdivided, we have to recompute colors etc.

    std::map<int, int> componentIDs;                          // stores the componentID for each triangle. Used for coloring
    std::map<std::set<int>, std::set<int>> adjFacesToEdge;    // faces adjacent to an edge given by two vertices
    std::map<std::set<int>, std::set<int>> adjVerticesToEdge; // vertices adjacent to an edge given by two vertices
    std::map<int, std::set<int>> adjVerticesToVertex;         // vertices adjacent to a vertex
    std::map<int, std::set<int>> adjFacesToVertex;            // indices of faces that are adjacent to a given vertex
    std::map<int, std::set<int>> adjFacesToFace;              // indices of faces that are adjacent to a given face

    int enext(const int t_orTri);
    int sym(const int t_orTri);
    int org(const int t_orTri);
    int dest(const int t_orTri);


    void initAdjacencyLists(); // Builds fnext and all adjacency data structures

    void computeNumberOfComponents();

    bool orientTriangles();

    void drawEdges();
};
