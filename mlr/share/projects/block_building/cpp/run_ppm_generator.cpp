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

#include "glibc_hotfix.cpp"

using namespace std;


void printOutGFile(mlr::KinematicWorld& outKinWorld, string outFilePath) {
    cout << "Printing file: " << outFilePath << endl;
    FILE(STRING(outFilePath)) << outKinWorld;
}


int main(int argc,char** argv){
    int argv_index_counter = 1;

    string filePath = argv[argv_index_counter++];
    string outFilePath = argv[argv_index_counter++];
    string imageFilePath = argv[argv_index_counter++];

    bool saveImage = false;
    istringstream(argv[argv_index_counter++]) >> saveImage;

    mlr::KinematicWorld kinWorld;
    kinWorld.init(STRING(filePath));

    printOutGFile(kinWorld, outFilePath);

    if (saveImage) {
        // viewing at an angle
//        kinWorld.gl().camera.setPosition(-3.5, -2.0, 2.5);
//        kinWorld.gl().camera.focus(3.5, 1.8, 0.);

        // viewing straight zoomed out
//        kinWorld.gl().camera.setPosition(-5.5,0,2.7);
//        kinWorld.gl().camera.focus(2.3,0,1.);

        // viewing straight zoomed in -- precariousness
//        kinWorld.gl().camera.setPosition(2.5,0,1.24);
//        kinWorld.gl().camera.focus(-2.4,0,1.);

        // cfs, stable-unstable pairs viewing angle and causal history
//        kinWorld.gl().camera.setPosition(2.2,0,1.4);
//        kinWorld.gl().camera.focus(-1.,0,1.);
//        kinWorld.gl().camera.setHeightAngle(18.);  // for the cfs wider angle

        // blocks paper visualization
        kinWorld.gl().camera.setPosition(-2.6,0,2.1);
        kinWorld.gl().camera.focus(1.,0,1.);

        kinWorld.gl().camera.upright();
//        kinWorld.gl().width = 1500;
//        kinWorld.gl().height = 1000;
        kinWorld.gl().width = 3000;  // for the cfs wider angle
        kinWorld.gl().height = 2000;  // for the cfs wider angle

        kinWorld.qdim.clear();
        kinWorld.watch(false);
        kinWorld.qdim.clear();
        write_ppm(kinWorld.gl().captureImage, imageFilePath.c_str(), true);
    }

    return 0;
}
