//
//  mesh.cpp
//  CS3242 Lab 1
//
//  Created by Bastian Morath on 04.04.19.
//  Copyright © 2019 NUS. All rights reserved.
//

#include "mesh.h"
#include "helper.h"
#include "math.h"
#include "loopSubdivision.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <Eigen/Dense>

using std::cout;
using std::endl;

/**
 * @descWrites current object to a file
 * @param bool smooth - Whether we should use smooth shading
 * @param bool edges - Whether we should draw the edges only
 * @param bool t_color_components - Whether we should color the different components
 */
void myObjType::draw(bool smooth, bool edges, bool t_color_components)
{

    glEnable(GL_LIGHTING);

    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_COLOR_MATERIAL);

    glPushMatrix();
    double longestSide = 0.0;
    for (int i = 0; i < 3; i++)
        if ((lmax[i] - lmin[i]) > longestSide)
            longestSide = (lmax[i] - lmin[i]);
    glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
    glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);


    // Make sure that those strings only get printed once (since this draw() method gets called every frame...)
    static bool stringInitialized;
    static std::string noEdges = "This object does not have any edges!";
    static std::string edgeDrawn = "Edges drawn!";

    if (edges && helper::objectHasEdges(fNextList, triangleList, tcount))
    {
        drawEdges();
        glDisable(GL_LIGHTING);
        glPopMatrix();
        if (!stringInitialized || !edgesDrawnAfterSubdivision)
        {
            cout << edgeDrawn << endl;
            edgesDrawnAfterSubdivision = true;
            stringInitialized = true;
        }
        return;
    }
    else if (edges && !helper::objectHasEdges(fNextList, triangleList, tcount))
    {

        if (!stringInitialized || !edgesDrawnAfterSubdivision)
        {
            cout << noEdges << endl;
            stringInitialized = true;
            edgesDrawnAfterSubdivision = true;
        }
    }

    for (int i = 1; i <= tcount; i++)
    {
        glBegin(GL_POLYGON);
        if (!smooth)
        {
            glNormal3dv(triangleNormalList[i]);
        }
        // Color each different component with a different color
        if (t_color_components)
        {
            glColor3f(colors[componentIDs[i]][0], colors[componentIDs[i]][1], colors[componentIDs[i]][2]);
        }
        else
        {
            glColor3f(0.3, 0.3, 0.3);
        }

        for (int j = 0; j < 3; j++)
        {
            if (smooth)
            {
                glNormal3dv(vertexNormalList[triangleList[i][j]]);
            }
            glVertex3dv(vList[triangleList[i][j]]);
        }
        glEnd();
    }

    glDisable(GL_LIGHTING);
    glutPostRedisplay(); // So that image is not black at the beginning

    glPopMatrix();
}

/**
 * @desc Writes current object to a file
 * @param str::string filename - name of object-file that curent object should be written to.
 *                               Can be .obj or .off!
 */
void myObjType::writeFile(std::string filename)
{
    if (filename.find(".off") != std::string::npos) {
        std::ostringstream lines;
        lines << "OFF" << endl;

        lines << vcount << " " << tcount << endl;
        for (int i = 1; i <= vcount; i++)
        {
            lines << std::to_string(vList[i][0]) << " " << std::to_string(vList[i][1]) << " " << std::to_string(vList[i][2]) << endl;
        }
        for (int i = 1; i <= tcount; i++)
        {
            lines << "3 " << std::to_string(triangleList[i][0] -1) << " " << std::to_string(triangleList[i][1] - 1) << " " << std::to_string(triangleList[i][2] -1) << endl;
        }

        std::ofstream outfile(filename);
        outfile << lines.str() << endl;
        
        outfile.close();
    } else if (filename.find(".obj") != std::string::npos) {
        std::ostringstream lines;
        for (int i = 1; i <= vcount; i++)
        {
            lines << "v " << std::to_string(vList[i][0]) << " " << std::to_string(vList[i][1]) << " " << std::to_string(vList[i][2]) << endl;
        }
        for (int i = 1; i <= tcount; i++)
        {
            lines << "f " << std::to_string(triangleList[i][0]) << " " << std::to_string(triangleList[i][1]) << " " << std::to_string(triangleList[i][2]) << endl;
        }
        
        std::ofstream outfile(filename);
        outfile << lines.str() << endl;
        
        outfile.close();
    } else {
        cout << "File format not supported " << filename << endl;
        exit(1);
    }
    
    cout << "Obejct successfully written to disk as ' " << filename << "'" << endl;

  
}

/**
 * @desc Opens a file, inits triangle and vertex lists, then inits adjacency lists, computes components, orients triangles and outputs some statistics
 * @param std::string filename - name of file that should be opened
 */
void myObjType::readFile(std::string filename) {
    if (filename.find(".off") != std::string::npos) {
        readFile_off(filename);
    } else if (filename.find(".obj") != std::string::npos) {
        readFile_obj(filename);
    } else {
        cout << "File format not supported " << filename << endl;
        exit(1);
    }
}


/**
 * @desc Opens a object file, inits triangle and vertex lists, then inits adjacency lists, computes components, orients triangles and outputs some statistics
 * @param std::string filename - name of object-file that should be opened
 */
void myObjType::readFile_obj(std::string filename)
{
    vcount = 0;
    tcount = 0;
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "_";
    cout << endl;
    cout << "Opening " << filename << endl;
    std::ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open())
    {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }

    std::string line;
    int i, j;
    bool firstVertex = 1;
    double currCood;

    while (getline(inFile, line))
    {
        if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ')
        {
            if (line[0] == 'v')
            {
                vcount++;
                i = 1;
                const char *linec = line.data();
                for (int k = 0; k < 3; k++)
                { // k is 0,1,2 for x,y,z
                    while (linec[i] == ' ')
                        i++;
                    j = i;
                    while (linec[j] != ' ')
                        j++;
                    currCood = vList[vcount][k] = atof(line.substr(i, j - i).c_str());
                    if (firstVertex)
                        lmin[k] = lmax[k] = currCood;
                    else
                    {
                        if (lmin[k] > currCood)
                            lmin[k] = currCood;
                        if (lmax[k] < currCood)
                            lmax[k] = currCood;
                    }
                    i = j;
                }
                firstVertex = 0;
            }
            if (line[0] == 'f')
            {
                tcount++;
                i = 1;
                const char *linec = line.data();
                for (int k = 0; k < 3; k++)
                {
                    while (linec[i] == ' ')
                        i++;
                    j = i;
                    while (linec[j] != ' ' && linec[j] != '\\')
                        j++;
                    triangleList[tcount][k] = atof(line.substr(i, j - i).c_str());
                    i = j;
                    fNextList[tcount][k] = 0;
                    while (linec[j] != ' ')
                        j++;
                }
            }
        }
    }
    
    cout << "Set up adjacency lists and fnext " << endl;
    initAdjacencyLists();
    
    cout << "Calculate Face Normals " << endl;
    helper::fillFaceNormals(triangleNormalList, vList, triangleList, tcount);
    
    cout << "Calculate Vertex Normals " << endl;
    helper::fillVertexNormals(vertexNormalList, triangleNormalList, adjFacesToVertex, vcount);
    
    cout << "Orienting Triangles..." << endl;
    orientTriangles();
    
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;
    
    cout << "Compute number of components..." << endl;
    computeNumberOfComponents();
    
    helper::computeStatistics(vList, vcount, triangleList, tcount);
    colors = std::vector< std::vector<double> >();
    for (int c = 0; c < numUniqueComponents; c++)
    {
        colors.push_back({((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX))});
    }
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "_";
    cout << endl;
}
#include <sstream>

/**
 * @desc Opens a.off file, inits triangle and vertex lists, then inits adjacency lists, computes components, orients triangles and outputs some statistics
 * Note: Compared to .obj files, in .off file, vertex indices start at 0!! -> Takes care of this
 * @param std::string filename - name of off-file that should be opened
 */
void myObjType::readFile_off(std::string filename)
{
    vcount = 0;
    tcount = 0;
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "_";
    cout << endl;
    cout << "Opening " << filename << endl;
    std::ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open())
    {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }
    
    std::string line;
    int i, j;
    bool firstVertex = 1;
    double currCood;
    int lineCount = 0;
    int numVertices = 0;
    int numFaces = 0;
    while (getline(inFile, line))
    {
        if (lineCount==0) {
            if (line.substr(0, 3) != "OFF") {
                cout << "This is not an OFF file! " << filename << endl;
                exit(1);
            }
        } else if (lineCount==1) {
            std::stringstream ss(line.c_str());
            int a, b;
            ss >> a >> b;
            numVertices = a;
            numFaces = b;
        } else {
            if (lineCount < numFaces + numVertices + 2) {
                if (lineCount <= numVertices + 1)  { // Vertex
                    vcount++;
                    i = 0;
                    const char *linec = line.data();
                    for (int k = 0; k < 3; k++)
                    { // k is 0,1,2 for x,y,z
                        while (linec[i] == ' ')
                            i++;
                        j = i;
                        while (linec[j] != ' ')
                            j++;
                        currCood = vList[vcount][k] = atof(line.substr(i, j - i).c_str());
                        if (firstVertex)
                            lmin[k] = lmax[k] = currCood;
                        else
                        {
                            if (lmin[k] > currCood)
                                lmin[k] = currCood;
                            if (lmax[k] < currCood)
                                lmax[k] = currCood;
                        }
                        i = j;
                    }
                    firstVertex = 0;
                    
                } else { // Faces
                    if (line.substr(0, 1) != "3") {
                        cout << "Program only accepts triangles!" << filename << endl;
                        exit(1);
                    }
                    tcount++;
                    i = 1;
                    const char *linec = line.data();
                    for (int k = 0; k < 3; k++)
                    {
                        while (linec[i] == ' ')
                            i++;
                        j = i;
                        while (linec[j] != ' ' && linec[j] != '\\')
                            j++;
                        
                        triangleList[tcount][k] = atof(line.substr(i, j - i).c_str()) + 1; // !! In .off file, vertex indices start at 1
                        i = j;
                        fNextList[tcount][k] = 0;
                        while (linec[j] != ' ')
                            j++;
                    }
                }
            }
        }
        lineCount++;
    }
    
    cout << "Set up adjacency lists and fnext " << endl;
    initAdjacencyLists();
    
    cout << "Calculate Face Normals " << endl;
    helper::fillFaceNormals(triangleNormalList, vList, triangleList, tcount);
    
    cout << "Calculate Vertex Normals " << endl;
    helper::fillVertexNormals(vertexNormalList, triangleNormalList, adjFacesToVertex, vcount);
    
    cout << "Orienting Triangles..." << endl;
    orientTriangles();
    
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;
    
    cout << "Compute number of components..." << endl;
    computeNumberOfComponents();
    
    helper::computeStatistics(vList, vcount, triangleList, tcount);
    colors = std::vector< std::vector<double> >();
    for (int c = 0; c < numUniqueComponents; c++)
    {
        colors.push_back({((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX))});
    }
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "_";
    cout << endl;
}



int myObjType::enext(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);

    int triangleIndex = t_orTri >> 3;

    std::map<int, int> my_map = {
        {0, 1},
        {1, 2},
        {2, 0},
        {3, 5},
        {4, 3},
        {5, 4}};

    return (triangleIndex << 3) | my_map[version];
}

int myObjType::sym(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);
    int triangleIndex = t_orTri >> 3;

    std::map<int, int> my_map = {
        {0, 3},
        {1, 4},
        {2, 5},
        {3, 0},
        {4, 1},
        {5, 2}};

    return (triangleIndex << 3) | my_map[version];
}

int myObjType::org(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);
    int triangleIndex = t_orTri >> 3;

    std::map<int, int> my_map = {
        {0, 0},
        {1, 1},
        {2, 2},
        {3, 1},
        {4, 2},
        {5, 0}};

    return triangleList[triangleIndex][my_map[version]];
}

int myObjType::dest(const int t_orTri)
{
    return org(sym(t_orTri));
}

/**
 * @desc Computes the number of components, which is defined by the number of independent surfaces, which are not shared by any triangle.
 
 For this we start at an arbitrary triangle, then enqueue the neighboring triangles and pop until no one are left. All those triangles will be the ones in the same component.
 We store every seen Index, and finish everything as soon as we have seen all indices.
 */
void myObjType::computeNumberOfComponents()
{
    componentIDs = {};
    numUniqueComponents = 0;
    std::set<int> seenIndices;
    std::queue<int> indicesToTraverse; // indices of triangles still to traverse

    while (seenIndices.size() < tcount) // One while loop corresponds to one component
    {
        int notSeenIndex = helper::getIndexNotYetSeen(tcount, seenIndices);
        indicesToTraverse.push(notSeenIndex);
        std::set<int> s;

        while (!indicesToTraverse.empty())
        {
            int idx = indicesToTraverse.front();
            indicesToTraverse.pop();
            componentIDs[idx] = numUniqueComponents;

            if (seenIndices.find(idx) == seenIndices.end())
            { // Triangle not yet seen
                seenIndices.insert(idx);

                for (auto &neighbor : adjFacesToFace[idx])
                {
                    indicesToTraverse.push(neighbor);
                }
            }
        }

        indicesToTraverse = {};
        numUniqueComponents++;
    }
    
   
    
    cout << "Number of Components: " << numUniqueComponents << endl;
}

// This namespace is used for  storing the intermediate values in the subdivision-schemes. at the end, they are copied to the global variables
namespace newSubdivision
{
    int vcount = 0;
    int tcount = 0;
    int triangleList[MAXT][3];
    double vList[MAXV][3];
    std::map<int, int> componentIDs; // Determines the componentID for each triangle. Used for coloring
} // namespace newSubdivision

/**
 * @desc Does barycentric subdivision, then updates all adjacency lists and recomputes number of components
 */
void myObjType::subdivideBarycentric()
{
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;

    cout << "Subdividing with Barycentric..." << endl;

    for (int i = 1; i <= tcount; i++) // For each triangle, we create 4 new triangles
    {
        std::vector<Eigen::Vector3d> newVertices;

        for (int version = 0; version < 3; version++) // Iterate over each pair of edge vertices
        {
            int edgeVertexIdx1 = triangleList[i][version];
            int edgeVertexIdx2 = triangleList[i][(version + 1) % 3];

            // 1. Compute one new edge vertex as midpoint of edge
            Eigen::Vector3d edgeVertex1(vList[edgeVertexIdx1][0], vList[edgeVertexIdx1][1], vList[edgeVertexIdx1][2]);
            Eigen::Vector3d edgeVertex2(vList[edgeVertexIdx2][0], vList[edgeVertexIdx2][1], vList[edgeVertexIdx2][2]);
            newVertices.push_back((edgeVertex1 + edgeVertex2) / 2.0);
        }

        // 2. Compute centroid
        Eigen::Vector3d v1(vList[triangleList[i][0]][0], vList[triangleList[i][0]][1], vList[triangleList[i][0]][2]);
        Eigen::Vector3d v2(vList[triangleList[i][1]][0], vList[triangleList[i][1]][1], vList[triangleList[i][1]][2]);
        Eigen::Vector3d v3(vList[triangleList[i][2]][0], vList[triangleList[i][2]][1], vList[triangleList[i][2]][2]);

        Eigen::Vector3d centroid = (v1 + v2 + v3) / 3.0;

        newVertices.push_back(v1);
        newVertices.push_back(v2);
        newVertices.push_back(v3);
        newVertices.push_back(centroid);

        // 4. Rebuild mesh / Connect vertices to create new faces
        std::vector<int> newVertexIndices;
        for (int j = 0; j < 7; j++)
        {
            std::pair<bool, int> result = loopSubdivision::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, newVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first)
            {
                newSubdivision::vcount++;
            }
        }

        // Add 6 triangles from using the new vertices
        Eigen::Vector3i t1(newVertexIndices[6], newVertexIndices[3], newVertexIndices[0]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t1);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t2(newVertexIndices[6], newVertexIndices[0], newVertexIndices[4]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t2);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t3(newVertexIndices[6], newVertexIndices[4], newVertexIndices[1]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t3);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t4(newVertexIndices[6], newVertexIndices[1], newVertexIndices[5]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t4);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t5(newVertexIndices[6], newVertexIndices[5], newVertexIndices[2]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t5);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t6(newVertexIndices[6], newVertexIndices[2], newVertexIndices[3]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t6);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
    }

    tcount = newSubdivision::tcount;
    vcount = newSubdivision::vcount;
    //componentIDs = newSubdivision::componentIDs;

    std::copy(&newSubdivision::triangleList[0][0], &newSubdivision::triangleList[0][0] + tcount * 3 + 3, &triangleList[0][0]);
    std::copy(&newSubdivision::vList[0][0], &newSubdivision::vList[0][0] + vcount * 3 + 3, &vList[0][0]);

    cout << "Subdivision completed... Recalculating data structures, normals etc. " << endl;

    initAdjacencyLists();
    helper::fillFaceNormals(triangleNormalList, vList, triangleList, tcount);
    helper::fillVertexNormals(vertexNormalList, triangleNormalList, adjFacesToVertex, vcount);
    // orientTriangles();
    computeNumberOfComponents(); // Don't add it!!!!

    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;

    newSubdivision::tcount = 0;
    newSubdivision::vcount = 0;
    subdivided = true;
    edgesDrawnAfterSubdivision = false;

    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;
}


/**
 * @desc Does barycentric subdivision, then updates all adjacency lists and recomputes number of components
 * @param int beta_version - Version 1:  beta = 3.0 / 8 / n;
 *                           Version 2:  beta = 1. / n * (5. / 8 - pow(3. / 8 + 1. / 4 * cos(2. * M_PI / n), 2));
 */
void myObjType::subdivideLoop(int beta_version)
{
    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;

    cout << "Subdividing with loop-subdivision..." << endl;

    for (int i = 1; i <= tcount; i++) // For each triangle, we create 4 new triangles
    {
        std::vector<Eigen::Vector3d> oddVertices;
        std::vector<Eigen::Vector3d> evenVertices;

        for (int version = 0; version < 3; version++) // Iterate over each pair of edge vertices
        {
            std::pair<int, int> edgeVertices = std::make_pair(triangleList[i][version], triangleList[i][(version + 1) % 3]);
            int edgeVertexIdx1 = edgeVertices.first;
            int edgeVertexIdx2 = edgeVertices.second;

            std::set<int> adjacentEdgeVertices = adjVerticesToEdge[{triangleList[i][version], triangleList[i][(version + 1) % 3]}];

            // 2. Compute one new odd vertex from the two edge vertices
            Eigen::Vector3d average;

            if (adjacentEdgeVertices.size() == 1)
            { // We have an adge with only one neighboring face
                average = loopSubdivision::getOddLoopVertexEdge(vList, edgeVertexIdx1, edgeVertexIdx2);
            }
            else
            { // We have an edge with two neighboring faces
                int adjIdx1 = *std::next(adjacentEdgeVertices.begin(), 0);
                int adjIdx2 = *std::next(adjacentEdgeVertices.begin(), 1);
                average = loopSubdivision::getOddLoopVertex(vList, edgeVertexIdx1, edgeVertexIdx2, adjIdx1, adjIdx2);
            }

            oddVertices.push_back(average);

            // 3. Compute one new even vertex from the first vertex of the edge
            std::set<int> adjacentVertexVertices = adjVerticesToVertex[edgeVertexIdx1];
            Eigen::Vector3d newVertex;
            if (adjacentVertexVertices.size() == 2)
            { // We have an adge with only one
                newVertex = loopSubdivision::getEvenLoopVertexEdge(vList, edgeVertexIdx1, *std::next(adjacentVertexVertices.begin(), 0), *std::next(adjacentVertexVertices.begin(), 1));
            }
            else
            {
                newVertex = loopSubdivision::getEvenLoopVertex(vList, edgeVertexIdx1, adjacentVertexVertices, beta_version);
            }
            evenVertices.push_back(newVertex);
        }

        // 4. Rebuild mesh / Connect vertices to create new faces
        std::vector<int> newVertexIndices;
        for (int j = 0; j < 3; j++)
        {
            std::pair<bool, int> result = loopSubdivision::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, oddVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first)
            {
                newSubdivision::vcount++;
            }
        }

        for (int j = 0; j < 3; j++)
        {
            std::pair<bool, int> result = loopSubdivision::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, evenVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first)
            {
                newSubdivision::vcount++;
            }
            componentIDs[result.second] = componentIDs[i];
        }

        // Add 4 triangles from using the new vertices
        Eigen::Vector3i t1(newVertexIndices[3], newVertexIndices[0], newVertexIndices[2]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t1);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t2(newVertexIndices[0], newVertexIndices[4], newVertexIndices[1]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t2);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t3(newVertexIndices[2], newVertexIndices[0], newVertexIndices[1]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t3);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
        Eigen::Vector3i t4(newVertexIndices[2], newVertexIndices[1], newVertexIndices[5]);
        loopSubdivision::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++, t4);
        //newSubdivision::componentIDs[newSubdivision::tcount] = componentIDs[i];
    }

    tcount = newSubdivision::tcount;
    vcount = newSubdivision::vcount;
    //componentIDs = newSubdivision::componentIDs;

    std::copy(&newSubdivision::triangleList[0][0], &newSubdivision::triangleList[0][0] + tcount * 3 + 3, &triangleList[0][0]);
    std::copy(&newSubdivision::vList[0][0], &newSubdivision::vList[0][0] + vcount * 3 + 3, &vList[0][0]);

    cout << "Subdivision completed... Recalculating data structures, normals etc. " << endl;

    initAdjacencyLists();
    helper::fillFaceNormals(triangleNormalList, vList, triangleList, tcount);
    helper::fillVertexNormals(vertexNormalList, triangleNormalList, adjFacesToVertex, vcount);
    orientTriangles();
    computeNumberOfComponents(); // Don't add it!!!!
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;

    newSubdivision::tcount = 0;
    newSubdivision::vcount = 0;
    subdivided = true;
    edgesDrawnAfterSubdivision = false;

    cout << endl;
    for (int i = 0; i < 50; i++)
        cout << "#";
    cout << endl;
}


/**
 * @desc Orients the triangles of the mesh. Idea: Start at an arbitraty triangle, then add all neighbors to a queue
 * and check their orientation - orient if ncessary -, subsequently processing all elements in the queue. If done,
 * we go to the next component (if any)

 * @return bool - Whether orientation was successfull
 */
bool myObjType::orientTriangles()
{
    std::set<int> seenIndices;
    std::queue<int> indicesToTraverse;
    int num_triangles_oriented = 0;
    while (seenIndices.size() < tcount) // One while loop corresponds to one component
    {
        int notSeenIndex = helper::getIndexNotYetSeen(tcount, seenIndices);
        indicesToTraverse.push(notSeenIndex);
        seenIndices.insert(notSeenIndex);

        while (!indicesToTraverse.empty())
        {
            int idx = indicesToTraverse.front();
            indicesToTraverse.pop();

            for (int version = 0; version < 3; version++)
            { // Check each neighbor
                int orTri_neighbor = fNextList[idx][version];
                if (orTri_neighbor != 0)
                { // If no edge vertex
                    int neighbor_index = orTri_neighbor >> 3;
                    int neighbor_version = orTri_neighbor & ((1 << 2) - 1);
                    bool hasConflict = helper::sameOrientation(idx, version, neighbor_index, neighbor_version, triangleList);
                    if (seenIndices.find(neighbor_index) != seenIndices.end())
                    { // Already seen
                        if (hasConflict)
                        {
                            cout << "Failure in orienting triangles...!" << endl;

                            return false;
                        }
                    }
                    else
                    {
                        if (hasConflict)
                        {
                            // Element not yet seen but conflict -> Orient it
                            int oldValue = triangleList[neighbor_index][1];
                            triangleList[neighbor_index][1] = triangleList[neighbor_index][2];
                            triangleList[neighbor_index][2] = oldValue;

                            // Also update fnext
                            int oldValueFnext = fNextList[neighbor_index][0];
                            fNextList[neighbor_index][0] = fNextList[neighbor_index][2];
                            fNextList[neighbor_index][2] = oldValueFnext;

                            num_triangles_oriented += 1;
                        }
                        indicesToTraverse.push(neighbor_index);
                        seenIndices.insert(neighbor_index);
                    }
                }
            }
        }
        indicesToTraverse = {};
    }

    if (num_triangles_oriented == 0)
    {
        cout << "No triangles had to be oriented!" << endl;
    }
    else
    {
        // Since triangles had to be flipped, we need to recompute the normals and also fnext

        cout << "Successfully oriented " << num_triangles_oriented << " triangles!" << endl;
        cout << "Datastructures have to be recomputed..." << endl;
        initAdjacencyLists();
        helper::fillFaceNormals(triangleNormalList, vList, triangleList, tcount);
        helper::fillVertexNormals(vertexNormalList, triangleNormalList, adjFacesToVertex, vcount);
    }

    return true;
}

/**
 * @desc Draws the edges of the object in red color, not showing the mesh itself
 */
void myObjType::drawEdges()
{
    glDisable(GL_LIGHTING);
    for (int i = 1; i <= tcount; i++)
    {
        // Check each triangle
        for (int version = 0; version < 3; version++)
        {
            int orTri_neighbor = fNextList[i][version];
            if (orTri_neighbor == 0)
            { // Edge vertices!
                std::pair<int, int> edgeVertices = helper::getVerticesForVersion(triangleList, i, version);
                glBegin(GL_LINES);
                glColor3f(1.0f, 0.0f, 0.0f); // make this vertex red
                glVertex3dv(vList[edgeVertices.first]);
                glVertex3dv(vList[edgeVertices.second]);
                glEnd();
            }
        }
    }
}

/**
 * @desc Inits fNextList, adjFacesToVertex, adjFacesToEdge, adjVerticesToEdge, adjVerticesToVertex and adjFacesToFace
 */
void myObjType::initAdjacencyLists()
{
    cout << "Init adjacency lists... " << endl;
    adjFacesToVertex = {};
    adjFacesToEdge = {};
    adjFacesToFace = {};
    adjVerticesToEdge = {};
    adjVerticesToVertex = {};

    // 1. Init adjFacesToEdge, adjVerticesToVertex and adjFacesToVertex
    // For an edge given by two vertices, store the adjacent faces
    for (int i = 1; i <= tcount; i++)
    {
        for (int version = 0; version < 3; version++)
        {
            int vIdx0 = triangleList[i][version];
            int vIdx1 = triangleList[i][(version + 1) % 3];
            int vIdx2 = triangleList[i][(version + 2) % 3];
            std::set<int> key = {vIdx0, vIdx1};
            int orTri = i << 3 | version;
            adjFacesToEdge[key].insert(orTri);
            adjVerticesToEdge[key].insert(vIdx2);

            adjVerticesToVertex[vIdx0].insert(vIdx1);
            adjVerticesToVertex[vIdx0].insert(vIdx2);
            adjFacesToVertex[vIdx0].insert(i);
        }
    }
    /*
    for(std::map<std::set<int>, std::set<int>>::const_iterator it = adjFacesToEdge.begin();
        it != adjFacesToEdge.end(); ++it)
    {
        int v0 = *std::next(it->first.begin(), 0);
        int v1 = *std::next(it->first.begin(), 1);
        int f0 = *std::next(it->second.begin(), 0);
        int f1 = *std::next(it->second.begin(), 1);
        cout << "Edge: {" << v0 << ",  " << v1 <<"} -> fnext:{idx: "
        <<  (f0 >> 3) << " , v: " << (f0 & ((1 << 2) - 1) ) << "}\n";
        cout << "Edge: {" << v0 << ",  " << v1 <<"} -> fnext:{idx: "
        <<  (f1 >> 3) << " , v: " << (f1 & ((1 << 2) - 1) ) << "}\n";
        
    }
  */
    // 2. Init adjFacesToFace
    for (int i = 1; i <= tcount; i++)
    {
        for (int version = 0; version < 3; version++)
        {
            std::set<int> key = {triangleList[i][version], triangleList[i][(version + 1) % 3]};

            std::set<int> opposite_faces = adjFacesToEdge[key];
            int face0 = *std::next(opposite_faces.begin(), 0);
            int face1 = *std::next(opposite_faces.begin(), 1);
            // If face1 index is not <=tcount, then this is not a face, but an edge face! -> Store 0
            if (opposite_faces.size() == 1)
            {
                face1 = 0;
            }

            int fnext = i == (face0 >> 3) ? face1 : face0; // Opposite face is the one that is not the current_face
            fNextList[i][version] = fnext;

            if (fnext != 0)
            { // If no edge vertex
                adjFacesToFace[i].insert(fnext >> 3);
            }
        }
    }
}
