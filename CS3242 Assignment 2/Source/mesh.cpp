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

using namespace std;



void myObjType::draw(bool smooth, bool edges) {
    
    glEnable(GL_LIGHTING);
    
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    
    glPushMatrix();
    double longestSide = 0.0;
    for (int i = 0; i < 3; i++)
        if ((lmax[i] - lmin[i]) > longestSide)
            longestSide = (lmax[i] - lmin[i]);
    glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
    glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
    
    
    if (edges){
        drawEdges();
    } else {
        for (int i = 1; i <= tcount; i++)
        {
            if (!smooth) {
                glNormal3dv(triangleNormalList[i]);
            }
            glBegin(GL_POLYGON);
            for (int j = 0; j < 3; j++){
                if (smooth) {
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

void myObjType::writeFile(const char* filename)
{
    ostringstream lines;
    for (int i=1; i <= vcount; i++) {
        lines << "v " << std::to_string(vList[i][0]) << " " << std::to_string(vList[i][1]) << " " << std::to_string(vList[i][2]) << endl;
    }
    for (int i=1; i <= tcount; i++) {
        lines << "f " << std::to_string(triangleList[i][0]) << " " << std::to_string(triangleList[i][1]) << " " << std::to_string(triangleList[i][2]) << endl;
    }
    
    std::ofstream outfile (filename);
    outfile << lines.str() << std::endl;
    
    outfile.close();
    
}

void myObjType::readFile(const char* filename)
{
    cout << "Opening " << filename << endl;
    ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open()) {
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
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ') j++;
                    currCood = vList[vcount][k] = atof(line.substr(i, j - i).c_str());
                    if (firstVertex)
                        lmin[k] = lmax[k] = currCood;
                    else {
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
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) {
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ' && linec[j] != '\\') j++;
                    triangleList[tcount][k] = atof(line.substr(i, j - i).c_str());
                    i = j;
                    fNextList[tcount][k] = 0;
                    while (linec[j] != ' ') j++;
                    
                }
                
            }
            
            
        }
    }
    calculateFaceNormals();
    calculateVertexNormals();
    computeFNextList();
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeAngleStatistics();
    computeNumberOfComponents();
    orientTriangles(); // TODO: Number of faces changed
}


/**
 * @desc Calculates the angle between two vectors
 * @param Vector3d t_tv1 - First vector
 * @param Vector3d t_tv2 - Second vector
 * @return double - Angle in Degrees
 */
double myObjType::calculateAngle(const Eigen::Vector3d t_v1, const Eigen::Vector3d t_v2){
    
    double dot = t_v1.dot(t_v2);
    
    return acos(dot / t_v1.norm() / t_v2.norm()) *180 / M_PI;
}

/**
 * @desc Calculates all face normals, using the cross product, and stores it in triangleNormalList
*/
void myObjType::calculateFaceNormals()
{
    // We suggest you to compute the normals here
    for (int i=1; i <= tcount; i++) {
        Eigen::Vector3d  v1(vList[triangleList[i][0]]);
        Eigen::Vector3d  v2(vList[triangleList[i][1]]);
        Eigen::Vector3d  v3(vList[triangleList[i][2]]);
        
        Eigen::Vector3d  v1_to_v2(v2 - v1);
        Eigen::Vector3d  v1_to_v3(v3 - v1);
        
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
    for (int i=1; i <= vcount; i++) {
        std::vector<int> adjacent_triangle_indices;
        for(int row=1;row<=tcount;++row)
            for(int column=0;column<3;++column)
                if(triangleList[row][column] == i)
                    adjacent_triangle_indices.push_back(row);
        
        Eigen::Vector3d sumVector(0, 0, 0);
        
        for (auto const& j: adjacent_triangle_indices)
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
        { 0, 1 },
        { 1, 2 },
        { 2, 0 },
        { 3, 5 },
        { 4, 3 },
        { 5, 4 }
    };
    
    return (triangleIndex << 3) | my_map[version];
}

int myObjType::sym(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);
    int triangleIndex = t_orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 3 },
        { 1, 4 },
        { 2, 5 },
        { 3, 0 },
        { 4, 1 },
        { 5, 2 }
    };
    
    return (triangleIndex << 3) | my_map[version];
}

int myObjType::org(const int t_orTri)
{
    int version = t_orTri & ((1 << 2) - 1);
    int triangleIndex = t_orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 0},
        { 1, 1 },
        { 2, 2 },
        { 3, 1 },
        { 4, 2 },
        { 5, 0 }
    };
    
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
    double minAngle = 0;
    double maxAngle = 0;
    
    for (int i=1; i <= tcount; i++) {
        Eigen::Vector3d  v1(vList[triangleList[i][0]]);
        Eigen::Vector3d  v2(vList[triangleList[i][1]]);
        Eigen::Vector3d  v3(vList[triangleList[i][2]]);
        
        Eigen::Vector3d  v1_to_v2(v2 - v1);
        Eigen::Vector3d  v1_to_v3(v3 - v1);
        Eigen::Vector3d  v2_to_v3(v3 - v2);
        
        double angle1 = calculateAngle(v1_to_v2, v1_to_v3);
        
        double angle2 = calculateAngle(-v1_to_v2, v2_to_v3);
        double angle3 = 180.0 - angle2 - angle1;
        
        double min = std::min(std::min(angle1, angle2), angle3);
        double max = std::max(std::max(angle1, angle2), angle3);
        
        statMinAngle[int(floor(min / 10))] += 1;
        statMaxAngle[int(floor(max / 10))] += 1;
        
        minAngle = minAngle < min ? minAngle : min;  // TODO: CHECK worng
        maxAngle = maxAngle > max ? maxAngle : max;
        
    }
    
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
}
/**
 * @desc Calculates fnext
 We first create a hashmap which stores, for each edge given by two vertex indices, the adjacent two faces. If it only has one face, because it is an edge triangle, then the value 0 is stored instead.
 
 We can then easily traverse the triangle list once and find the corresponding adjacent triangle index for each triangle version
 */
void myObjType::computeFNextList() {
    // Create hash_map, that takes edge vertices as key, and returns the two triangles opposite of it
    map<set<int>, set<int>> mymap;
    // mymap.insert(make_pair(make_pair(1,2), 3)); //edited
    
    for (int i=1; i <= tcount; i++) {
        
        int v[3] = {triangleList[i][0], triangleList[i][1], triangleList[i][2]};
        
        int v0 = v[0];
        int v1 = v[1];
        int v2 = v[2];
        
        int f0 =  i << 3 | 0;
        int f1 =  i << 3 | 1;
        int f2 =  i << 3 | 2;
        
        // f0
        std::set<int> key0 = {v0, v1};
        mymap[key0].insert(f0); //store old and new face
        
        
        // f1
        std::set<int> key1 = {v1, v2};
        mymap[key1].insert(f1); //store old and new face
        
        // f2
        std::set<int> key2 = {v0, v2};
        mymap[key2].insert(f2); //store old and new face
        
    }
    // Hashmap created
    
    for (int i=1; i <= tcount; i++) {
        
        for (int version = 0; version<3;version++) {
            std::set<int> key ={triangleList[i][version], triangleList[i][(version + 1) % 3]};
            
            std::set<int> opposite_faces = mymap[key];
            int face0 = *std::next(opposite_faces.begin(), 0);
            int face1 = *std::next(opposite_faces.begin(), 1);
            // If face1 index is not <=tcount, then this is not a face, but an edge face!
            face1 = (face1 >> 3) <= tcount ? face1 : 0;
            
            fNextList[i][version] = i == (face0 >> 3) ? face1 : face0; // Opposite face is the one that is not the current_face
        }
        
        
        
        //        std::cout << (fNextList[i][0] >> 3) << ", " << (fNextList[i][0] & ((1 << 2) - 1))
        //        << " | " <<(fNextList[i][1] >> 3) << ", " << (fNextList[i][1] & ((1 << 2) - 1))
        //        << " | " << (fNextList[i][2] >> 3) << ", " << (fNextList[i][2] &  ((1 << 2) - 1)) << std::endl;
        
    }
}

/**
 * @desc Computes the number of components, which is defined by the number of independent surfaces, which are not shared by any triangle.
 
 For this we start at an arbitrary triangle, and traverse each adjacent triangle (using fnext and while maintaining a list of all triangles already seen) recursively, until no ones are left.
 We then do the same with a triangle that we have not yet traversed (and thus have to be in another compoennt), until no triangles are left
 */
void myObjType::computeNumberOfComponents() {
    std::cout << "Computing number of components..." << std::endl;
    
    std::vector<set<int>> v; // bundles the triangle ids together that are in the same component
    set<int> seenIndices;
    
    while (seenIndices.size() < tcount) {
        set<int> s;
        v.push_back(s);
        int notSeenIndex = getIndexNotYetSeen(seenIndices);
        findNeighbors(v, seenIndices, notSeenIndex);
    }
    
    std::cout << "Number of Components: " << v.size() << std::endl;
}

/**
 * @desc Finds all neighbors of a triangle
 * @param vector<set<int>> &t_v - a vector containing a set of triangles for each different component
 * @param set<int> &t_seenIndices - All vertices that we have traversed so far. Used to terminate recursion
 * @param int t_index - Index of triangle that we should start our search from
 */
void myObjType::findNeighbors(std::vector<set<int>> &t_v, set<int> &t_seenIndices, const int t_index) {
    int numComponents = int(t_v.size());
    t_seenIndices.insert(t_index);
    t_v[numComponents-1].insert(t_index);
    for (int version=0; version <3; version++) {
        int orTri_neighbor = fNextList[t_index][version];
        if (orTri_neighbor != 0) { // If no edge vertex
            int neighbor_index = orTri_neighbor >> 3;
            if (t_v[numComponents-1].find(neighbor_index) == t_v[numComponents-1].end()) { // Element not yet seen
                findNeighbors(t_v, t_seenIndices, neighbor_index);
            }
        }
    }
    
}

/**
 * @desc Finds a triangle index (from 1 to tcount) that is not in t_v. Returns the smallest one
 * @param set<int> t_v - Set of indices that have already been seen/traversed
 * @return int - Smallest such index
 */
int myObjType::getIndexNotYetSeen(const set<int> t_v) {
    for (int i=1; i<= tcount; i++){
        if (t_v.find(i) == t_v.end()){
            return i;
        }
    }
    return -1;
}


bool myObjType::orientTriangles() {
    std::cout << "Orienting Triangles..." << std::endl;
    
    std::set<int> seenIndices;
    
    bool success;
    while (seenIndices.size() < tcount) {
        int notSeenIndex = getIndexNotYetSeen(seenIndices);
        std::set<int> currentComponentIds = {notSeenIndex};
        seenIndices.insert(notSeenIndex);
        success = checkOrientationIndex(notSeenIndex, currentComponentIds, seenIndices);
        if (!success) {
            std::cout << "Failure in orienting triangles!" << std::endl;
            return false;
        }
    }
    computeFNextList();
    calculateFaceNormals();
    calculateVertexNormals();
    
    std::cout << "Successfully oriented triangles!" << std::endl;
    
    return true;
}

bool myObjType::checkOrientationIndex(const int t_index, std::set<int> &t_currentComponentIds, std::set<int> &t_seenIndices) {
    for (int version=0; version <3; version++) { // Check each neighbor
        int orTri_neighbor = fNextList[t_index][version];
        int neighbor_index = orTri_neighbor >> 3;
        if (orTri_neighbor != 0) { // If no edge vertex
            
            int neighbor_version = orTri_neighbor & ((1 << 2) - 1);
            bool hasConflict = conflict(t_index, version, neighbor_index, neighbor_version);
            if (t_currentComponentIds.find(neighbor_index) != t_currentComponentIds.end()) { // Element already seen
                if (hasConflict) {
                    return false;
                }
            } else {
                if (hasConflict) {
                    // Element not yet seen but conflict -> Orient it
                    int oldValue = triangleList[neighbor_index][1];
                    triangleList[neighbor_index][1] = triangleList[neighbor_index][2];
                    triangleList[neighbor_index][2] = oldValue;
                }
                t_currentComponentIds.insert(neighbor_index);
                t_seenIndices.insert(neighbor_index);
                
                checkOrientationIndex(neighbor_index, t_currentComponentIds, t_seenIndices);
            }
        }
    }
    
    return true;
}

bool myObjType::conflict(const int t_t1Index, const int t_t1Version, const int t_t2Index, const int t_t2Version) {
    pair<int, int> t1Vertices = getVerticesForVersion(t_t1Index, t_t1Version);
    pair<int, int> t2Vertices = getVerticesForVersion(t_t2Index, t_t2Version);
    return t1Vertices == t2Vertices;
}


pair<int, int> myObjType::getVerticesForVersion(const int t_triangleIndex, const int t_version) {
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
    return make_pair(v0, v1);
}

void myObjType::drawEdges() {
    //std::cout << "Drawing Edges..." << std::endl;
    static bool initialized;
    static std::set<std::pair<int, int>> edgeVerticesSet;
    
    if (!initialized) {
        for (int i=1; i <= tcount; i++) {
            // Check each triangle
            for (int version=0; version <3; version++) {
                int orTri_neighbor = fNextList[i][version];
                if (orTri_neighbor == 0) { // Edge vertices!
                    pair<int, int> edgeVertices = getVerticesForVersion(i, version);
                    edgeVerticesSet.insert(make_pair(edgeVertices.first, edgeVertices.second));
                }
            }
        }
        initialized = true;
    }
    
    // Default : lighting
    glDisable(GL_LIGHTING);
    
    for (auto& edge: edgeVerticesSet){
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f); // make this vertex red
        glVertex3dv(vList[edge.first]);
        glVertex3dv(vList[edge.second]);
        glEnd();
        
    }
    
    // Default : lighting
    glEnable(GL_LIGHTING);
}
