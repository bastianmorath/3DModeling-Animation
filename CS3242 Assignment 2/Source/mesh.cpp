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



void myObjType::draw(bool smooth) {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
	for (int i = 1; i <= tcount; i++)
	{
        // uncomment the following after you computed the normals
        if (smooth) {
            glBegin(GL_POLYGON);
            for (int j = 0; j < 3; j++){
                glNormal3dv(vnlist[tlist[i][j]]);
                glVertex3dv(vlist[tlist[i][j]]);
            }
            glEnd();
        } else {
            glBegin(GL_POLYGON);
            glNormal3dv(nlist[i]);
            for (int j = 0; j < 3; j++)
                glVertex3dv(vlist[tlist[i][j]]);
            glEnd();
        }
	}
	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void myObjType::writeFile(char* filename)
{
    ostringstream lines;
    for (int i=1; i <= vcount; i++) {
        lines << "v " << std::to_string(vlist[i][0]) << " " << std::to_string(vlist[i][1]) << " " << std::to_string(vlist[i][2]) << endl;
    }
    for (int i=1; i <= tcount; i++) {
        lines << "f " << std::to_string(tlist[i][0]) << " " << std::to_string(tlist[i][1]) << " " << std::to_string(tlist[i][2]) << endl;
    }
    
    std::ofstream outfile (filename);
    outfile << lines.str() << std::endl;

    outfile.close();
    
}

void myObjType::readFile(char* filename)
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
					currCood = vlist[vcount][k] = atof(line.substr(i, j - i).c_str());
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
					tlist[tcount][k] = atof(line.substr(i, j - i).c_str());
					i = j;
					fnlist[tcount][k] = 0;
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
    computeStat();
    computeNumberOfComponents();
}



double myObjType::calculateAngle(Eigen::Vector3d v1, Eigen::Vector3d v2){
    
    double dot = v1.dot(v2);
    
    return acos(dot / v1.norm() / v2.norm()) *180 / M_PI;
}


void myObjType::calculateFaceNormals()
{
    // We suggest you to compute the normals here
    for (int i=1; i <= tcount; i++) {
        Eigen::Vector3d  v1(vlist[tlist[i][0]]);
        Eigen::Vector3d  v2(vlist[tlist[i][1]]);
        Eigen::Vector3d  v3(vlist[tlist[i][2]]);
        
        Eigen::Vector3d  v1_to_v2(v2 - v1);
        Eigen::Vector3d  v1_to_v3(v3 - v1);
        
        Eigen::Vector3d crossP = v1_to_v2.cross(v1_to_v3);
        crossP.normalize();
        
        nlist[i][0] = crossP[0];
        nlist[i][1] = crossP[1];
        nlist[i][2] = crossP[2];
    }
}

void myObjType::calculateVertexNormals()
{
    // We suggest you to compute the normals here
    for (int i=1; i <= vcount; i++) {
        std::vector<int> adjacent_triangle_indices;
        
        for(int column=1;column<=3;++column)
            for(int row=1;row<=tcount;++row)
                if(tlist[row][column] == i)
                    adjacent_triangle_indices.push_back(row);
        
        Eigen::Vector3d sumVector(0, 0, 0);
        
        for (auto const& j: adjacent_triangle_indices)
        {
            sumVector[0] += nlist[j][0];
            sumVector[1] += nlist[j][1];
            sumVector[2] += nlist[j][2];
        }
        sumVector.normalize();
        
        vnlist[i][0] = sumVector[0];
        vnlist[i][1] = sumVector[1];
        vnlist[i][2] = sumVector[2];
    }
}


int myObjType::enext(int orTri)
{
    int version = orTri & ((1 << 2) - 1);
    // std::cout << version << std::endl;

    int triangleIndex = orTri >> 3;
    
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

int myObjType::sym(int orTri)
{
    int version = orTri & ((1 << 2) - 1);
    int triangleIndex = orTri >> 3;
    
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

int myObjType::org(int orTri)
{
    int version = orTri & ((1 << 2) - 1);
    int triangleIndex = orTri >> 3;
    
    std::map<int, int> my_map = {
        { 0, 0},
        { 1, 1 },
        { 2, 2 },
        { 3, 1 },
        { 4, 2 },
        { 5, 0 }
    };
    
    return tlist[triangleIndex][my_map[version]];
}

int myObjType::dest(int orTri)
{
    return org(sym(orTri));
}


// PRIVATE MEMBERS


void myObjType::computeStat()
{
    double minAngle = 0;
    double maxAngle = 0;
    
    for (int i=1; i <= tcount; i++) {
        Eigen::Vector3d  v1(vlist[tlist[i][0]]);
        Eigen::Vector3d  v2(vlist[tlist[i][1]]);
        Eigen::Vector3d  v3(vlist[tlist[i][2]]);
        
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
        
        minAngle = minAngle < min ? minAngle : min;
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

void myObjType::computeFNextList() {
    // Create hash_map, that takes edge vertices as key, and returns the two triangles opposite of it
    map<pair<int, int>, pair<int, int>> mymap;
    // mymap.insert(make_pair(make_pair(1,2), 3)); //edited
    
    for (int i=1; i <= tcount; i++) {

        int v[3] = {tlist[i][0], tlist[i][1], tlist[i][2]};
        std::vector<int> v_vec (v, v+3);
        std::sort(std::begin(v_vec), std::end(v_vec));
        
        int v0 = v_vec[0];
        int v1 = v_vec[1];
        int v2 = v_vec[2];

        int f0 =  i << 3 | int(distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][0])));
        int f1 =  i << 3 | int(distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][1])));
        int f2 =  i << 3 | int(distance(v_vec.begin(), find(v_vec.begin(), v_vec.end(), tlist[i][2])));
        
        // f0
        std::pair<int, int> key0 = make_pair(v0, v1);
        if (mymap.count(key0) != 0) { // One face already stored
            mymap[key0] = make_pair(mymap[key0].first, f0); //store old and new face
        } else {
            mymap.insert(make_pair(key0, make_pair(f0, 0))); //store new face, leave the other blank, i.e. 0
        }
        
        // f1
        std::pair<int, int> key1 = make_pair(v1, v2);
        if (mymap.count(key1) != 0) { // One face already stored
            mymap[key1] = make_pair(mymap[key1].first, f1); //store old and new face
        } else {
            mymap.insert(make_pair(key1, make_pair(f1, 0))); //store new face, leave the other blank, i.e. 0
        }
        
        // f2
        std::pair<int, int> key2 = make_pair(v0, v2);
        if (mymap.count(key2) != 0) { // One face already stored
            mymap[key2] = make_pair(mymap[key2].first, f2); //store old and new face
        } else {
            mymap.insert(make_pair(key2, make_pair(f2, 0))); //store new face, leave the other blank, i.e. 0
        }
    } // Hashmap created
    /*
    for(auto& elem : mymap)
    {
        std::cout << "{" << elem.first.first << ",  " << elem.first.second << "}: {" <<  (elem.second.first >> 3) << "," << (elem.second.second >> 3) << "}\n";
    }
    */
    for (int i=1; i <= tcount; i++) {
        int o = min(tlist[i][0], tlist[i][1]) ;
        int d = max(tlist[i][0], tlist[i][1]) ;
        // std::cout << "Org.: " << tlist[i][0] << ", Dest: " <<  tlist[i][1] << "\n";

         // int current_face =  i << 3 | 0;
      
        std::pair<int, int> opposite_faces = mymap[make_pair(o, d)];
        //std::cout << "{TrObj: " << (current_face) << ", index:  " << (current_face >>3) << ", version: " << (current_face & ((1 << 2) - 1)
        // )  << ". Opposite: " << (opposite_faces.first) << std::endl;
        fnlist[i][0] = i == (opposite_faces.first >> 3) ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
        
        o = min(tlist[i][1], tlist[i][2]);
        d = max(tlist[i][1], tlist[i][2]);
        
        opposite_faces = mymap[make_pair(o, d)];

        fnlist[i][1] = i == (opposite_faces.first >> 3) ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
       
        o = min(tlist[i][0], tlist[i][2]);
        d = max(tlist[i][0], tlist[i][2]);
        
        opposite_faces = mymap[make_pair(o, d)];
        fnlist[i][2] = i == (opposite_faces.first >> 3) ? opposite_faces.second : opposite_faces.first; // Opposite face is the one that is not the current_face
    }
    
}


void myObjType::computeNumberOfComponents() {
    std::cout << "Computing number of components..." << std::endl;
    
    std::vector<set<int>> neighboring_ids; // Each triagnle has a vector wioth all the triangles visited
    for (int i=1; i <= tcount; i++) {
        std::set<int> new_set = {i};
        neighboring_ids.push_back(new_set);
    }
    // We bacially need to do it as many times as the biggest distance between any two triangles is
    // SO for an approximation and to speed things up, I use the logarithm of total number of triangles
    for (int i=0; i < log(tcount); i++) {
        for (int j=0; j < tcount; j++) {
            for (int version=0; version <3; version++) {
                int orTri_neighbor = fnlist[j+1][version];
                if (orTri_neighbor != 0) { // If no edge vertex
                    int neighbor_index = orTri_neighbor >> 3;
                    set<int> set = neighboring_ids[neighbor_index-1];
                    neighboring_ids[i].insert(set.begin(), set.end());
                } 
            }
        }
    }
    
    std::vector<set<int>> component_ids; // We store all ids of a component in a separate vector
    for (int i=0; i < neighboring_ids.size(); i++) {
        pair<bool, int> intersection(false, 0);
        for (int j=0; j < component_ids.size(); j++) {
            std::set<int> component_ids_set = neighboring_ids[i];
            std::set<int> id_vec = component_ids[j];
            
            
            std::vector<int> v3;
            std::set_intersection(component_ids_set.begin(),component_ids_set.end(),
                                  id_vec.begin(),id_vec.end(),
                                  back_inserter(v3));
            if (v3.size() > 0) {
                intersection = make_pair(true, j);
                break;
            }
        }
        
        if (intersection.first) {
            std::set<int> toBeInserted = neighboring_ids[i];
            component_ids[intersection.second].insert(toBeInserted.begin(), toBeInserted.end());
        } else {
            component_ids.push_back(neighboring_ids[i]);
        }
    }

    std::cout << "Number of Components: " <<  component_ids.size() << std::endl;
}

/*
int myObjType::getNumberOfComponents() {
    std::vector<int> triangle_ids;
    for (int i=1; i <= tcount; i++) {
        triangle_ids.push_back(i);
    }

    
    // Do with version 0
    std::vector<int> previousIndices = {1};
    for (int i=1; i <= tcount; i++) {
        changeNeighbors(previousIndices, 1, triangle_ids);
    }
        
    return distance(triangle_ids.begin(),unique(triangle_ids.begin(), triangle_ids.end()));

    
    return 0;
}
 */
/*
 Idea: Recursively change the ids of the three neighboring triangles to the current id.
 
 IDEA 2: Each triangle has a list of indices. We update each triangle at the same time to include ids of neighboring triangle (i.e. two nested for loops). at the end, all the triangles that have any idshared are in the same component. 
 */
void myObjType::changeNeighbors(std::vector<int> previous_indices, int currentIndex, std::vector<int> triangle_ids) {
    for (int version=0; version <3; version++) {
        int orTri_neighbor = fnlist[currentIndex][version];
        int neighbor_index = orTri_neighbor >> 3;
        // If we have not looked at this triagnle before in this current run
       

        if (std::find(std::begin(previous_indices), std::end(previous_indices), neighbor_index) == std::end(previous_indices)) {
            //for (auto i = triangle_ids.begin(); i != triangle_ids.end(); ++i)
                // std::cout << *i << ' ';
            // std::cout << std::endl;
            triangle_ids[neighbor_index] = triangle_ids[currentIndex]; // Change the fnext ids to this id
            previous_indices.push_back(neighbor_index);
            changeNeighbors(previous_indices, neighbor_index, triangle_ids);
        }
    }
}


std::vector<int> myObjType::mergeUnique(std::vector<int> vector1, std::vector<int> vector2) {
    std::vector<int> merge;
    merge.reserve( vector1.size() + vector2.size() ); // preallocate memory
    merge.insert( merge.end(), vector1.begin(), vector1.end() );
    merge.insert( merge.end(), vector2.begin(), vector2.end() );
    sort( merge.begin(), merge.end() );
    merge.erase( unique( merge.begin(), merge.end() ), merge.end() );
    return merge;
}
