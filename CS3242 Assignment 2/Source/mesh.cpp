#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include "math.h"
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "mesh.h"
#include <map>
#include <queue>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <array>
#include <set>
#include <ctime>
#include "helper.h"

using namespace std;

void myObjType::draw(bool smooth, bool edges)
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
  
    static bool initialized;
    
    static vector<vector<double>> colors;
    
    if (!initialized || subdivided) {
        for (int c=0;c < numUniqueComponents;c++) {
            colors.push_back({((double) rand() / (RAND_MAX)), ((double) rand() / (RAND_MAX)), ((double) rand() / (RAND_MAX))});
        }
        subdivided = false;
        initialized = true;
    }

    if (edges)
    {
        drawEdges();
    }
    else
    {
        for (int i = 1; i <= tcount; i++)
        {
            
            if (!smooth)
            {
                glNormal3dv(triangleNormalList[i]);
            }
            glBegin(GL_POLYGON);
            // Color each different component with a different color
            glColor3f(colors[componentIDs[i]][0], colors[componentIDs[i]][1], colors[componentIDs[i]][2]);
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
    }
    glDisable(GL_LIGHTING);
    glutPostRedisplay(); // So that image is not black at the beginning

    glPopMatrix();
}

void myObjType::writeFile(const char *filename)
{
    ostringstream lines;
    for (int i = 1; i <= vcount; i++)
    {
        lines << "v " << std::to_string(vList[i][0]) << " " << std::to_string(vList[i][1]) << " " << std::to_string(vList[i][2]) << endl;
    }
    for (int i = 1; i <= tcount; i++)
    {
        lines << "f " << std::to_string(triangleList[i][0]) << " " << std::to_string(triangleList[i][1]) << " " << std::to_string(triangleList[i][2]) << endl;
    }
    
    std::ofstream outfile(filename);
    outfile << lines.str() << std::endl;

    outfile.close();
}

void myObjType::readFile(const char *filename)
{
    cout << "Opening " << filename << endl;
    ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open())
    {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }

    string line;
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
   
    
    initAdjacencyLists();
    

    
    cout << "Calculate Face Normals " << endl;

    calculateFaceNormals();
    cout << "Calculate Vertex Normals " << endl;
    calculateVertexNormals();
    cout << "Calculate fNext list " << endl;
    computeFNextList();
    
    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
    std::cout << "Compute number of components..." << std::endl;
    computeNumberOfComponents();
    
    
    std::cout << "Orienting Triangles..." << std::endl;
    orientTriangles();

   
    computeAngleStatistics();
    cout << endl;

    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    cout << endl;
    
}


/**
 * @desc Calculates all face normals, using the cross product, and stores it in triangleNormalList
 */
void myObjType::calculateFaceNormals()
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
/**
 * @desc Calculates all vertex normals, using the average of all adjacent faces, and stores it in vertexNormalList
 */
void myObjType::calculateVertexNormals()
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

int myObjType::enext(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);
    // std::cout << version << std::endl;
    
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

// PRIVATE MEMBERS

/**
 * @desc Computes the angles in all triangles and counts how many times they fall into each 10-degree angle bin
 */
void myObjType::computeAngleStatistics()
{
    double minAngle = 360;
    double maxAngle = 0;
    
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
    for( int i=0; i< 50; i++) cout << "#";
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
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
}
/**
 * @desc Calculates fnext
 We first create a hashmap which stores, for each edge given by two vertex indices, the adjacent two faces. If it only has one face, because it is an edge triangle, then the value 0 is stored instead.
 
 We can then easily traverse the triangle list once and find the corresponding adjacent triangle index for each triangle version
 */
void myObjType::computeFNextList()
{
    for (int i = 1; i <= tcount; i++)
    {
        
        for (int version = 0; version < 3; version++)
        {
            std::set<int> key = {triangleList[i][version], triangleList[i][(version + 1) % 3]};
            
            std::set<int> opposite_faces = adjFacesToEdge[key];
            int face0 = *std::next(opposite_faces.begin(), 0);
            int face1 = *std::next(opposite_faces.begin(), 1);
            // If face1 index is not <=tcount, then this is not a face, but an edge face! -> Store 0
            if (opposite_faces.size() == 1) {
                face1 = 0;
            }
            
            int fnext = i == (face0 >> 3) ? face1 : face0; // Opposite face is the one that is not the current_face
            
            //std::cout << "Edge: {" << triangleList[i][version] << ",  " << triangleList[i][(version + 1) % 3] <<"} -> fnext:{idx: "
            //<<  (fnext >> 3) << " , v: " << (fnext & ((1 << 2) - 1) ) << "}\n";
            
            fNextList[i][version] = fnext;
        }
        
         // std::cout << (triangleList[i][0]) << ", "  << (triangleList[i][1]) << ", "<< (triangleList[i][2]) << ", "<< std::endl;
    }
}

/**
 * @desc Computes the number of components, which is defined by the number of independent surfaces, which are not shared by any triangle.
 
 For this we start at an arbitrary triangle, then enqueue the neighboring triangles and pop until no one are left. All those triangles will be the ones in the same component.
 We store every seen Index, and finish everything as soon as we have seen all indices.
 */
void myObjType::computeNumberOfComponents()
{
    numUniqueComponents = 0;
    std::vector<set<int>> v; // bundles the triangle ids together that are in the same component
    set<int> seenIndices;
    queue<int> indicesToTraverse; // indices of triangles still to traverse
    
    while (seenIndices.size() < tcount) // One while loop corresponds to one component
    {
        int notSeenIndex = helper::getIndexNotYetSeen(tcount, seenIndices);
        indicesToTraverse.push(notSeenIndex);
        set<int> s;
        v.push_back(s);
        
        while(!indicesToTraverse.empty()) {
            int idx = indicesToTraverse.front();
            indicesToTraverse.pop();
            componentIDs[idx] = numUniqueComponents;

            if (seenIndices.find(idx) == seenIndices.end()) { // Triangle not yet seen
                
                v[numUniqueComponents].insert(idx);
                seenIndices.insert(idx);

                for (auto& neighbor: adjFacesToFace[idx]) {
                    indicesToTraverse.push(neighbor >> 3);
                }
            }
        }
        
        indicesToTraverse = {};
        numUniqueComponents++;
    }
   
    std::cout << "Number of Components: " << numUniqueComponents << std::endl;
    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
}


namespace newSubdivision {
    int vcount = 0;
    int tcount = 0;
    int triangleList[MAXT][3];
    double vList[MAXV][3];
}


void myObjType::subdivideBarycentric(){
    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
    std::cout << "Subdividing with Barycentric..." << std::endl;

    for (int i = 1; i <= tcount; i++) // For each triangle, we create 4 new triangles
    {
        std::vector<Eigen::Vector3d> newVertices;
        Eigen::Vector3d centroid;
        
        for (int version = 0; version < 3; version++) // Iterate over each pair of edge vertices
        {
            std::pair<int, int> edgeVertices = std::make_pair(triangleList[i][version], triangleList[i][(version + 1) % 3]);
            int edgeVertexIdx1 = edgeVertices.first;
            int edgeVertexIdx2 = edgeVertices.second;
            
            // 1. Compute one new edge vertex as midpoint
            
            Eigen::Vector3d edgeVertex1(vList[edgeVertexIdx1][0], vList[edgeVertexIdx1][1], vList[edgeVertexIdx1][2]);
            Eigen::Vector3d edgeVertex2(vList[edgeVertexIdx2][0], vList[edgeVertexIdx2][1], vList[edgeVertexIdx2][2]);
            newVertices.push_back((edgeVertex1 + edgeVertex2) / 2.0 );
        }
        
        // 2. Compute centroid
        Eigen::Vector3d v1(vList[triangleList[i][0]][0], vList[triangleList[i][0]][1], vList[triangleList[i][0]][2]);
        Eigen::Vector3d v2(vList[triangleList[i][1]][0], vList[triangleList[i][1]][1], vList[triangleList[i][1]][2]);
        Eigen::Vector3d v3(vList[triangleList[i][2]][0], vList[triangleList[i][2]][1], vList[triangleList[i][2]][2]);
        
        centroid = (v1 + v2 + v3) / 3.0;
        
        newVertices.push_back(v1);
        newVertices.push_back(v2);
        newVertices.push_back(v3);
        newVertices.push_back(centroid);
        
        
        // 4. Rebuild mesh / Connect vertices to create new faces
        vector<int> newVertexIndices;
        for (int j=0;j<7;j++) {
            pair<bool, int> result = helper::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, newVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first) {
                newSubdivision::vcount++;
            }
            componentIDs[result.second] = componentIDs[i];
        }
        
        // Add 4 triangles from using the new vertices
        Eigen::Vector3i t1(newVertexIndices[6], newVertexIndices[3], newVertexIndices[0]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t1);
        Eigen::Vector3i t2(newVertexIndices[6], newVertexIndices[0], newVertexIndices[4]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t2);
        Eigen::Vector3i t3(newVertexIndices[6], newVertexIndices[4], newVertexIndices[1]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t3);
        Eigen::Vector3i t4(newVertexIndices[6], newVertexIndices[1], newVertexIndices[5]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t4);
        Eigen::Vector3i t5(newVertexIndices[6], newVertexIndices[5], newVertexIndices[2]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t5);
        Eigen::Vector3i t6(newVertexIndices[6], newVertexIndices[2], newVertexIndices[3]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t6);
    }
    
    tcount = newSubdivision::tcount;
    vcount = newSubdivision::vcount;
    
    for(int a = 1; a <= tcount; a++) {
        for(int b = 0; b < 3 + 1; b++) {
            triangleList[a][b] = newSubdivision::triangleList[a][b];
        }
    }
    for(int a = 1; a <= vcount; a++) {
        for(int b = 0; b < 3; b++) {
            vList[a][b] = newSubdivision::vList[a][b];
        }
    }
    
    initAdjacencyLists();
  
    computeFNextList();
    // orientTriangles()
    computeNumberOfComponents();
    calculateFaceNormals();
    calculateVertexNormals();
   
    newSubdivision::tcount = 0;
    newSubdivision::vcount = 0;
    subdivided = true;
    edgesDrawnAfterSubdivision = false;

    
    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
    
}



void myObjType::subdivideLoop()
{
    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;
    std::cout << "Subdividing with loop-subdivision..." << std::endl;

    
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
            if (adjacentEdgeVertices.size() == 1) { // We have an adge with only one neighboring face
                average = helper::getOddLoopVertexEdge(vList, edgeVertexIdx1, edgeVertexIdx2);
            } else { // We have an edge with two neighboring faces
                int adjIdx1 = *std::next(adjacentEdgeVertices.begin(), 0);
                int adjIdx2 = *std::next(adjacentEdgeVertices.begin(), 1);
                average = helper::getOddLoopVertex(vList, edgeVertexIdx1, edgeVertexIdx2, adjIdx1, adjIdx2);
            }
            oddVertices.push_back(average);
            
            
            // 3. Compute one new even vertex from the first vertex of the edge
            std::set<int> adjacentVertexVertices = adjVerticesToVertex[edgeVertexIdx1];
            Eigen::Vector3d newVertex;
            if (adjacentVertexVertices.size() == 2) { // We have an adge with only one
                newVertex = helper::getEvenLoopVertexEdge(vList, edgeVertexIdx1, *std::next(adjacentVertexVertices.begin(), 0), *std::next(adjacentVertexVertices.begin(), 1));
            } else {
                newVertex = helper::getEvenLoopVertex(vList, edgeVertexIdx1, adjacentVertexVertices);
            }
            
            evenVertices.push_back(newVertex);
        }
        
        // 4. Rebuild mesh / Connect vertices to create new faces
        vector<int> newVertexIndices;
        for (int j=0;j<3;j++) {
            pair<bool, int> result = helper::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, oddVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first) {
                newSubdivision::vcount++;
            }
        }
        for (int j=0;j<3;j++) {
            pair<bool, int> result = helper::addVertexToVertexList(newSubdivision::vList, newSubdivision::vcount, evenVertices[j]);
            newVertexIndices.push_back(result.second);
            if (result.first) {
                newSubdivision::vcount++;
            }
            componentIDs[result.second] = componentIDs[i];

        }
        
        // Add 4 triangles from using the new vertices
        Eigen::Vector3i t1(newVertexIndices[3], newVertexIndices[0], newVertexIndices[2]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t1);
        Eigen::Vector3i t2(newVertexIndices[0], newVertexIndices[4], newVertexIndices[1]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t2);
        Eigen::Vector3i t3(newVertexIndices[2], newVertexIndices[0], newVertexIndices[1]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t3);
        Eigen::Vector3i t4(newVertexIndices[2], newVertexIndices[1], newVertexIndices[5]);
        helper::addTriangleToTriangleList(newSubdivision::triangleList, newSubdivision::tcount++,  t4);
    }
    
    tcount = newSubdivision::tcount;
    vcount = newSubdivision::vcount;
    
    for(int a = 1; a <= tcount; a++) {
        for(int b = 0; b < 3 + 1; b++) {
            triangleList[a][b] = newSubdivision::triangleList[a][b];
        }
    }
    for(int a = 1; a <= vcount; a++) {
        for(int b = 0; b < 3; b++) {
            vList[a][b] = newSubdivision::vList[a][b];
        }
    }
    
    initAdjacencyLists();
    calculateFaceNormals();
    calculateVertexNormals();
    computeFNextList();
    computeNumberOfComponents();
    
    newSubdivision::tcount = 0;
    newSubdivision::vcount = 0;
    
    subdivided = true;
    edgesDrawnAfterSubdivision = false;

    cout << endl;
    for( int i=0; i< 50; i++) cout << "#";
    cout << endl;

}


bool myObjType::orientTriangles() {
    set<int> seenIndices;
    queue<int> indicesToTraverse;
    int num_triangles_oriented = 0;
    while (seenIndices.size() < tcount) // One while loop corresponds to one component
    {
        int notSeenIndex = helper::getIndexNotYetSeen(tcount, seenIndices);
        indicesToTraverse.push(notSeenIndex);
        seenIndices.insert(notSeenIndex);

        while(!indicesToTraverse.empty()) {
            int idx = indicesToTraverse.front();
            indicesToTraverse.pop();

            for (int version = 0; version < 3; version++)
            { // Check each neighbor
                int orTri_neighbor = fNextList[idx][version];
                if (orTri_neighbor != 0)
                { // If no edge vertex
                    int neighbor_index = orTri_neighbor >> 3;
                    int neighbor_version = orTri_neighbor & ((1 << 2) - 1);
                    bool hasConflict = conflict(idx, version, neighbor_index, neighbor_version);
                    if (seenIndices.find(neighbor_index) != seenIndices.end()) { // Already seen
                        if (hasConflict)
                        {
                            std::cout << "Failure in orienting triangles...!" << std::endl;

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
        std::cout << "No triangles had to be oriented!" << std::endl;
    }
    else
    {
        // Since triangles had to be flipped, we need to recompute the normals and also fnext
      
        std::cout << "Successfully oriented " << num_triangles_oriented << " triangles!" << std::endl;
        std::cout << "Datastructures have to be recomputed..." << std::endl;
        initAdjacencyLists();
        calculateFaceNormals();
        calculateVertexNormals();
        computeFNextList();
    }
    
    return true;

}

bool myObjType::conflict(const int t_t1Index, const int t_t1Version, const int t_t2Index, const int t_t2Version)
{
    pair<int, int> t1Vertices = helper::getVerticesForVersion(triangleList, t_t1Index, t_t1Version);
    pair<int, int> t2Vertices = helper::getVerticesForVersion(triangleList, t_t2Index, t_t2Version);
    return t1Vertices == t2Vertices;
}



void myObjType::drawEdges()
{
    static bool initialized;
    static std::set<std::pair<int, int>> edgeVerticesSet;

    if (!initialized || !edgesDrawnAfterSubdivision)
    {
        for (int i = 1; i <= tcount; i++)
        {
            // Check each triangle
            for (int version = 0; version < 3; version++)
            {
                int orTri_neighbor = fNextList[i][version];
                if (orTri_neighbor == 0)
                { // Edge vertices!
                    pair<int, int> edgeVertices = helper::getVerticesForVersion(triangleList, i, version);
                    edgeVerticesSet.insert(make_pair(edgeVertices.first, edgeVertices.second));
                }
            }
        }
        initialized = true;
    }

    static bool stringInitialized;
    static string noEdges = "This object does not have any edges!";
    static string edgeDrawn = "Edges drawn!";


    if (edgeVerticesSet.empty())
    {
        if (!stringInitialized || !edgesDrawnAfterSubdivision)
        {
            std::cout << noEdges << std::endl;
            stringInitialized = true;
        }
    }
    else
    {
        if (!stringInitialized || !edgesDrawnAfterSubdivision)
        {
            std::cout << edgeDrawn << std::endl;
            stringInitialized = true;
        }

        // Default : lighting
        glDisable(GL_LIGHTING);

        for (auto &edge : edgeVerticesSet)
        {
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f); // make this vertex red
            glVertex3dv(vList[edge.first]);
            glVertex3dv(vList[edge.second]);
            glEnd();
        }

        // Default : lighting
        glEnable(GL_LIGHTING);
    }
}

void myObjType::initAdjacencyLists()
{
    cout << "Init adjacency lists... " << endl;
    adjFacesToVertex = {};
    adjFacesToEdge = {};
    adjVerticesToEdge = {};
    adjVerticesToVertex = {};

    
    // 1. Init adjFacesToEdge, adjVerticesToVertex and adjFacesToVertex
    // For an edge given by two vertices, store the adjacent faces
    for (int i = 1; i <= tcount; i++)
    {
        Eigen::Vector3i v(triangleList[i][0], triangleList[i][1], triangleList[i][2]);

        int f0 = i << 3 | 0;
        int f1 = i << 3 | 1;
        int f2 = i << 3 | 2;

        // f0
        std::set<int> key0 = {v[0], v[1]};
        adjFacesToEdge[key0].insert(f0);

        // f1
        std::set<int> key1 = {v[1], v[2]};
        adjFacesToEdge[key1].insert(f1);

        // f2
        std::set<int> key2 = {v[0], v[2]};
        adjFacesToEdge[key2].insert(f2);
        
        // Init
        adjVerticesToVertex[v[0]].insert(v[1]);
        adjVerticesToVertex[v[0]].insert(v[2]);
        
        adjVerticesToVertex[v[1]].insert(v[0]);
        adjVerticesToVertex[v[1]].insert(v[2]);
        
        adjVerticesToVertex[v[2]].insert(v[0]);
        adjVerticesToVertex[v[2]].insert(v[1]);
        
        // Init
        adjFacesToVertex[v[0]].insert(i);
        adjFacesToVertex[v[1]].insert(i);
        adjFacesToVertex[v[2]].insert(i);
        
        
     
    }

    // 2. Init adjVerticesToEdge
    // For an edge given by two vertices, store the adjacent one or two vertices
    for (auto &elem : adjFacesToEdge)
    {
        std::set<int> edgeVertices(elem.first.begin(), elem.first.end());
        std::set<int> orTriFaces(elem.second.begin(), elem.second.end());

        std::set<int> vertices;
        for (auto &faceIdx : orTriFaces)
        {
            int idx = faceIdx >> 3;
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 0).first);
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 0).second);
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 1).first);
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 1).second);
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 2).first);
            vertices.insert(helper::getVerticesForVersion(triangleList, idx, 2).second);
        }
        std::set_difference(vertices.begin(), vertices.end(), edgeVertices.begin(), edgeVertices.end(),
                            std::inserter(adjVerticesToEdge[edgeVertices], adjVerticesToEdge[edgeVertices].end()));
    }
    
    // 3. Init adjFacesToFace
    for (int i = 1; i <= tcount; i++)
    {
        
        for (int version = 0; version < 3; version++)
        {
            std::set<int> key = {triangleList[i][version], triangleList[i][(version + 1) % 3]};
            
            std::set<int> opposite_faces = adjFacesToEdge[key];
            int face0 = *std::next(opposite_faces.begin(), 0);
            int face1 = *std::next(opposite_faces.begin(), 1);
            // If face1 index is not <=tcount, then this is not a face, but an edge face! -> Store 0
            if (opposite_faces.size() == 1) {
                face1 = 0;
            }
            
            int fnext = i == (face0 >> 3) ? face1 : face0; // Opposite face is the one that is not the current_face
            
            if (fnext != 0) { // If no edge vertex
                
                adjFacesToFace[i].insert(fnext);
            }
            
            
        }
    }
}
