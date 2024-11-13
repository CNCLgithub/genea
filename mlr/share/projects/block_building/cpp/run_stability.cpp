#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <Motion/komo.h>
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <Ors/ors_physx.h>

#define WATCH_STABILITY 1
#define WATCH_OUTPUT 0


using namespace std;


const char PLAN_SEPARATOR = '#';


void printOutGFile(mlr::KinematicWorld& outKinWorld, string outFilePath) {
    #if WATCH_OUTPUT
    cout << "Printing file: " << outFilePath << endl;
    #endif
    FILE(STRING(outFilePath)) << outKinWorld;
}


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


void simulateWorld(mlr::KinematicWorld& kinWorld, vector<string>& blockNamesList) {

    double seconds = 2.;

    kinWorld.gl().camera.setPosition(-5.,-1.,2.);
    kinWorld.gl().camera.focus(0,0,1.);
    kinWorld.gl().camera.upright();

    for(mlr::Body *b:kinWorld.bodies){

        b->type = mlr::BT_kinematic;

        string blockName(b->name);
        if(!isStringInList(blockNamesList, blockName)) continue;
        b->type = mlr::BT_dynamic;
        for(mlr::Joint *j:b->inLinks) j->type=mlr::JT_free;
    }

    kinWorld.qdim.clear();

    #if WATCH_STABILITY
    kinWorld.watch(true, "BEFORE PhysX -- press ENTER");
    #endif

    for(uint i = 0; i < seconds * 10; i++){
        kinWorld.physx().step(.005);
    }

    for(uint i = 0; i < seconds * 90; i++){
        kinWorld.physx().step(.005);
        #if WATCH_STABILITY
        kinWorld.watch();
//        string filename = "/home/jakiroshah/Desktop/state_" + to_string(i) + ".g";
//        printOutGFile(kinWorld, filename);
        #endif
    }

    for(mlr::Body *b:kinWorld.bodies){
        string blockName(b->name);
        if(isStringInList(blockNamesList, blockName)) {
            b->type = mlr::BT_kinematic;
            for(mlr::Joint *j:b->inLinks) j->type=mlr::JT_rigid;
        }
    }

    kinWorld.qdim.clear();
}



int main(int argc,char** argv){
    int argv_index_counter = 1;

    string filePath = argv[argv_index_counter++];
    string blockNamesString = argv[argv_index_counter++];  // names of all the blocks in the plan
    string outFilePath = argv[argv_index_counter++];

    bool doRunSimulation = false;
    istringstream(argv[argv_index_counter++]) >> doRunSimulation;

    KOMO komoObject;
    komoObject.verbose = 0;

    mlr::KinematicWorld kinWorld;
    initializeWorkSpace(komoObject, kinWorld, filePath);

    vector<string> blockNamesList;
    split(blockNamesString, PLAN_SEPARATOR, blockNamesList);

    if (doRunSimulation) simulateWorld(kinWorld, blockNamesList);
    if (outFilePath.length() > 2) printOutGFile(kinWorld, outFilePath);

    return 0;
}

