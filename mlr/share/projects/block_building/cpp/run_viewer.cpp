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


using namespace std;


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


int main(int argc,char** argv){
    int argv_index_counter = 1;

    string filePath = argv[argv_index_counter++];

    KOMO komoObject;
    komoObject.verbose = 0;

    mlr::KinematicWorld kinWorld;
    initializeWorkSpace(komoObject, kinWorld, filePath);

    kinWorld.gl().camera.setPosition(-5.,-1.,2.);
    kinWorld.gl().camera.focus(0,0,1.);
    kinWorld.gl().camera.upright();
    kinWorld.qdim.clear();
    kinWorld.watch(true);
    kinWorld.qdim.clear();

    return 0;
}

