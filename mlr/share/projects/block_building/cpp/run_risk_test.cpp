#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <random>

#include <chrono>
#include <thread>

#include <Motion/komo.h>
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <Ors/ors_physx.h>
#include <Motion/taskMaps.h>

#define WATCH_STABILITY 0
#define WATCH_OUTPUT 0

#define NOISE_THRESHOLD 0.018


using namespace std;


const char PLAN_SEPARATOR = '#';


bool isStringInList(vector<string>& inputList, string element) {
    if(find(begin(inputList), end(inputList), element) == end(inputList)) return false;
    return true;
}


void split(const string &inputString, char delimiter, vector<string> &elements) {
    stringstream inputStringStream(inputString);

    string item;
    while (getline(inputStringStream, item, delimiter)) {
        elements.push_back(item);
    }
}


void printOutDataFile(bool isUnstable, string outFilePath) {
    ofstream outputStream;
    outputStream.open(outFilePath, ios_base::trunc);
    outputStream << isUnstable << endl;
    outputStream.close();
}


void initializeWorkSpace(KOMO& komoObject, mlr::KinematicWorld& kinWorld, string filePath) {

    kinWorld.init(STRING(filePath));

    komoObject.setModel(kinWorld);

    komoObject.setTiming(10., 20, 5., 2, true);
    komoObject.setSquaredFixJointVelocities(-1., -1., 1e3);
    komoObject.setSquaredFixSwitchVelocities();
    komoObject.setSquaredQAccelerations();

    komoObject.displayCamera().setPosition(-5., -1., 2.);
    komoObject.displayCamera().focus(0, 0, 1.);
    komoObject.displayCamera().upright();

    // explicitly active certain collision computations (by SWIFT)
    komoObject.MP->world.swift().deactivate(komoObject.MP->world.getShapeByName("table"));
}


bool simulateWorld(mlr::KinematicWorld& kinWorld, vector<string>& blockNamesList) {

    double seconds = 2.;

    kinWorld.gl().camera.setPosition(-5.,-1.,2.);
    kinWorld.gl().camera.focus(0,0,1.);
    kinWorld.gl().camera.upright();

    vector<string> blockNamesVec;
    vector<double> xInitVec;
    vector<double> yInitVec;
    vector<double> zInitVec;

    vector<double> xNextVec;
    vector<double> yNextVec;
    vector<double> zNextVec;

    for(mlr::Body* b:kinWorld.bodies){

        b->type = mlr::BT_kinematic;

        string blockName(b->name);
        if(!isStringInList(blockNamesList, blockName)) continue;

        blockNamesVec.push_back(blockName);
        xInitVec.push_back(b->X.pos.x);
        yInitVec.push_back(b->X.pos.y);
        zInitVec.push_back(b->X.pos.z);

        b->type = mlr::BT_dynamic;
        for(mlr::Joint* j:b->inLinks) j->type=mlr::JT_free;

        if(blockName == "waist") {
            kinWorld.getBodyByName(b->name)->X.pos.z = 10;
        }
    }

    kinWorld.qdim.clear();

    #if WATCH_STABILITY
    kinWorld.watch(true);
    #endif

    for(uint i = 0; i < seconds * 10; i++){
        kinWorld.physx().step(.005);
    }

    for(uint i = 0; i < seconds * 90; i++){
        kinWorld.physx().step(.005);
        #if WATCH_STABILITY
        kinWorld.watch();
        #endif
    }

    for(mlr::Body* b:kinWorld.bodies){
        string blockName(b->name);

        if(!isStringInList(blockNamesList, blockName)) continue;

        b->type = mlr::BT_kinematic;
        for(mlr::Joint* j:b->inLinks) j->type=mlr::JT_rigid;

        xNextVec.push_back(b->X.pos.x);
        yNextVec.push_back(b->X.pos.y);
        zNextVec.push_back(b->X.pos.z);

        if(blockName == "waist") {
            kinWorld.getBodyByName(b->name)->X.pos.z = 1;
        }
    }

    kinWorld.qdim.clear();

    for(uint i = 0; i<zInitVec.size(); i++){
        if(!isStringInList(blockNamesList, blockNamesVec[i])) continue;
        bool condition1 = std::abs(xInitVec[i] - xNextVec[i]) > NOISE_THRESHOLD;
        bool condition2 = std::abs(yInitVec[i] - yNextVec[i]) > NOISE_THRESHOLD;
        bool condition3 = std::abs(zInitVec[i] - zNextVec[i]) > NOISE_THRESHOLD;
        if (condition1 || condition2 || condition3) {
            return true;
        }
    }

    return false;
}


int main(int argc,char** argv){
    int argv_index_counter = 1;

    string filePath = argv[argv_index_counter++];
    string blockNamesString = argv[argv_index_counter++];  // names of all the blocks in the plan
    string outputFilePath = argv[argv_index_counter++];

    KOMO komoObject;
    komoObject.verbose = 0;

    mlr::KinematicWorld kinWorld;
    initializeWorkSpace(komoObject, kinWorld, filePath);

    vector<string> blockNamesList;
    split(blockNamesString, PLAN_SEPARATOR, blockNamesList);

    bool isUnstable = simulateWorld(kinWorld, blockNamesList);

    #if WATCH_OUTPUT
    if (isUnstable) cout << "UNSTABLE" << endl;
    else cout << "STABLE" << endl;
    #endif

    printOutDataFile(isUnstable, outputFilePath);

    return 0;
}

