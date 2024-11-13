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

#define WATCH_ANIMATION 0
#define WATCH_STABILITY 0
#define WATCH_OUTPUT 0
#define SNAPSHOT_TIME 30

#define CENTER_LOC 0.0
#define PLACE_LOC_X 0.5
#define PLACE_LOC1_Y 0.3
#define PLACE_LOC2_Y -0.3

#define TABLE_X_LIMIT_MIN -0.15
#define TABLE_X_LIMIT_MAX 0.5
#define TABLE_Y_LIMIT_MIN -0.6
#define TABLE_Y_LIMIT_MAX 0.6
#define TABLE_HEIGHT_BLOCKS 0.9471
#define EPSILON_BLOCKS 0.025

#define NOISE_THRESHOLD 0.018


using namespace std;


const char PLAN_SEPARATOR = '#';
const char COMMANDS_SEPARATOR = ',';

const string TABLE_LEFT = "TL";
const string TABLE_RIGHT = "TR";

const string MOVE_GRAB_LEFT = "GL";
const string MOVE_GRAB_RIGHT = "GR";
const string MOVE_PLACE_LEFT = "PL";
const string MOVE_PLACE_RIGHT = "PR";
const string MOVE_FIX_LEFT = "FL";
const string MOVE_FIX_RIGHT = "FR";


class IntervalSet {
    std::map<float, float> _intervals;

    public:
        void add(float smaller, float bigger) {
            _intervals[smaller] = bigger;
        }

        const std::map<float, float>& Intervals() const { return _intervals; }

        bool isInsideIntervals(float queryValue) const {
            if (_intervals.empty()) return false;
            const auto suspectNext = _intervals.upper_bound(queryValue);
            const auto suspect = std::prev(suspectNext);
            return suspect->first <= queryValue && queryValue <= suspect->second;
        }
};


bool isStringInList(vector<string>& inputList, string element) {
    if(find(begin(inputList), end(inputList), element) == end(inputList)) return false;
    return true;
}


float generateRandomFloat(float mean, float min_val, float max_val) {
    random_device randomDevice;
    mt19937 randomGenerator(randomDevice());
    normal_distribution<float> normalDist(mean, 5 * EPSILON_BLOCKS);

    float sample = normalDist(randomGenerator);
    while (sample < min_val || sample > max_val) {
        sample = normalDist(randomGenerator);
    }

    return sample;
}


float getRandomFloatUniform(float min_val, float max_val) {
    return ((max_val - min_val) * ((float) rand() / RAND_MAX)) + min_val;
}


void split(const string &inputString, char delimiter, vector<string> &elements) {
    stringstream inputStringStream(inputString);

    string item;
    while (getline(inputStringStream, item, delimiter)) {
        elements.push_back(item);
    }
}


void printOutGFile(mlr::KinematicWorld& outKinWorld, string outFilePath) {
    FILE(STRING(outFilePath)) << outKinWorld;
}


void printOutDataFile(float outData, bool isUnstable, string outFilePath) {
    ofstream outputStream;
    outputStream.open(outFilePath, ios_base::trunc);
    outputStream << outData << ", " << isUnstable << endl;
    outputStream.close();
}


void moveBlockInWorld(mlr::KinematicWorld& kinWorld, const char* blockName, const char* surfaceName,
                      mlr::Transformation& finalPose) {
    mlr::Transformation rel = 0;
    mlr::KinematicSwitch* newKinSwitch = mlr::KinematicSwitch::newSwitch("rigidZero", surfaceName,
                                                                         blockName, kinWorld, 0,
                                                                         rel, finalPose);
    newKinSwitch->apply(kinWorld);
    delete newKinSwitch;
}


void initializeWorkSpace(KOMO& komoObject, mlr::KinematicWorld& initKinWorld,
                         mlr::KinematicWorld& finalKinWorld, mlr::KinematicWorld& ogFinalKinWorld,
                         string startFilePath, string finalFilePath, string ogFinalFilePath) {

    initKinWorld.init(STRING(startFilePath));
    finalKinWorld.init(STRING(finalFilePath));
    ogFinalKinWorld.init(STRING(ogFinalFilePath));

    komoObject.setModel(initKinWorld);

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


void setupKOMO(KOMO& komoObject, mlr::KinematicWorld& initKinWorld, mlr::KinematicWorld& finKinWorld, string plan,
               double noiseXLeft, double noiseYLeft, double noiseXRight, double noiseYRight,
               vector<int>& outCheckpointTimesList) {

    float timer = .2;

    vector<string> commandsList;
    split(plan, PLAN_SEPARATOR, commandsList);

    outCheckpointTimesList.push_back(timer * 20 + 24);  // adding checkpoint for the first grab

    for(string command : commandsList) {
        vector<string> robotMovesList;
        split(command, COMMANDS_SEPARATOR, robotMovesList);

        if(robotMovesList.size() == 0) {
            throw std::runtime_error("ERROR [setupKOMO]: invalid command encountered");
        }

        // increment timer for each command
        timer += .8;
        outCheckpointTimesList.push_back(timer * 20 + 24);

        for(string robotMove : robotMovesList) {
            vector<string> moveDetailsList;
            split(robotMove, '_', moveDetailsList);

            string moveName = moveDetailsList[0];
            const char* blockName = moveDetailsList[1].c_str();

            if(robotMovesList.size() == 0) {
                throw std::runtime_error("ERROR [setupKOMO]: invalid robot move encountered");
            }

            if(moveName == MOVE_GRAB_LEFT) {
                komoObject.setGrasp(timer, "humanL", blockName, 0, .8);
            } else if (moveName == MOVE_GRAB_RIGHT) {
                komoObject.setGrasp(timer, "humanR", blockName, 0, .8);
            } else if (moveName == MOVE_PLACE_LEFT) {
                string tableSide = moveDetailsList[2];
                if (tableSide == TABLE_LEFT) {
                    komoObject.setPlaceOne(timer, "humanL", blockName, "table");
                } else {
                    komoObject.setPlaceTwo(timer, "humanL", blockName, "table");
                }
            } else if (moveName == MOVE_PLACE_RIGHT) {
                string tableSide = moveDetailsList[2];
                if (tableSide == TABLE_LEFT) {
                    komoObject.setPlaceOne(timer, "humanR", blockName, "table");
                } else {
                    komoObject.setPlaceTwo(timer, "humanR", blockName, "table");
                }
            } else if (moveName == MOVE_FIX_LEFT) {
                const char* surfaceName = moveDetailsList[2].c_str();

                finKinWorld.getBodyByName(blockName)->X.pos.x += noiseXLeft;
                finKinWorld.getBodyByName(blockName)->X.pos.y += noiseYLeft;

                mlr::Transformation finalPose = finKinWorld.getBodyByName(blockName)->X /
                                                finKinWorld.getBodyByName(surfaceName)->X;

                moveBlockInWorld(finKinWorld, blockName, surfaceName, finalPose);

                komoObject.setPlaceFixed(timer, "humanL", blockName, surfaceName, finalPose);
            } else if (moveName == MOVE_FIX_RIGHT) {
                const char* surfaceName = moveDetailsList[2].c_str();

                finKinWorld.getBodyByName(blockName)->X.pos.x += noiseXRight;
                finKinWorld.getBodyByName(blockName)->X.pos.y += noiseYRight;

                mlr::Transformation finalPose = finKinWorld.getBodyByName(blockName)->X /
                                                finKinWorld.getBodyByName(surfaceName)->X;

                moveBlockInWorld(finKinWorld, blockName, surfaceName, finalPose);

                komoObject.setPlaceFixed(timer, "humanR", blockName, surfaceName, finalPose);
            }
        }
    }

    // increment timer for each command
    timer += + .8;
    outCheckpointTimesList.push_back(timer * 20 + 24);

    komoObject.reset();
    komoObject.run(1);

    #if WATCH_ANIMATION
    unsigned current_index = 0;
    for(uint timeT = 1; timeT < komoObject.MP->T; timeT++){
        if(timeT % 5 == 0 && timeT < 30) {
            mlr::KinematicWorld kinWorld = *komoObject.MP->configurations(timeT + komoObject.MP->k_order);

            kinWorld.gl().camera.setPosition(-5.,-1.,2.);
            kinWorld.gl().camera.focus(0,0,1.);
            kinWorld.gl().camera.upright();
            kinWorld.qdim.clear();
            kinWorld.watch(true);

//            string filename = "/home/jakiroshah/Desktop/state_" + to_string(timeT) + ".g";
//            printOutGFile(kinWorld, filename);
        }
//        if(timeT == outCheckpointTimesList[current_index]) {
//
//            mlr::KinematicWorld kinWorld = *komoObject.MP->configurations(timeT + komoObject.MP->k_order);
//
//            kinWorld.gl().camera.setPosition(-5.,-1.,2.);
//            kinWorld.gl().camera.focus(0,0,1.);
//            kinWorld.gl().camera.upright();
//            kinWorld.qdim.clear();
//            kinWorld.watch(true);
//
//            current_index += 1;
//
//            if (current_index >= outCheckpointTimesList.size()) break;
//        }
    }
    #endif
}


bool simulateWorld(KOMO& komoObject, mlr::KinematicWorld& kinWorld, mlr::KinematicWorld& finKinWorld, bool isDiff,
                   vector<string>& blockNamesList, vector<string>& grabbedBlockNamesList,
                   vector<string>& displacedBlockNamesList, vector<string>& restructureBlockNamesList) {

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
        if(!isStringInList(grabbedBlockNamesList, blockName)) {
            string block_name(b->name);
            blockNamesVec.push_back(block_name);
            xInitVec.push_back(b->X.pos.x);
            yInitVec.push_back(b->X.pos.y);
            zInitVec.push_back(b->X.pos.z);

            b->type = mlr::BT_dynamic;
            for(mlr::Joint* j:b->inLinks) j->type=mlr::JT_free;
        }

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

        if(!isStringInList(grabbedBlockNamesList, blockName)) {
            xNextVec.push_back(b->X.pos.x);
            yNextVec.push_back(b->X.pos.y);
            zNextVec.push_back(b->X.pos.z);
        }

        if(blockName == "waist") {
            kinWorld.getBodyByName(b->name)->X.pos.z = 1;
        }
    }

    kinWorld.qdim.clear();

    bool isUnstableFlag = false;

    for(uint i = 0; i<zInitVec.size(); i++){
        if(!isStringInList(blockNamesList, blockNamesVec[i])) continue;
        bool condition1 = std::abs(xInitVec[i] - xNextVec[i]) > NOISE_THRESHOLD;
        bool condition2 = std::abs(yInitVec[i] - yNextVec[i]) > NOISE_THRESHOLD;
        bool condition3 = std::abs(zInitVec[i] - zNextVec[i]) > NOISE_THRESHOLD;
        if (condition1 || condition2 || condition3) {  // check if blocks have moved substantially

            if (isDiff) {
                double xFin = finKinWorld.getBodyByName(blockNamesVec[i].c_str())->X.pos.x;
                double yFin = finKinWorld.getBodyByName(blockNamesVec[i].c_str())->X.pos.y;
                double zFin = finKinWorld.getBodyByName(blockNamesVec[i].c_str())->X.pos.z;

                condition1 = std::abs(xFin - xNextVec[i]) > NOISE_THRESHOLD;
                condition2 = std::abs(yFin - yNextVec[i]) > NOISE_THRESHOLD;
                condition3 = std::abs(zFin - zNextVec[i]) > NOISE_THRESHOLD;

                if (condition1 || condition2 || condition3) {  // check if moved blocks are at final position
                    isUnstableFlag = true;
                    #if WATCH_OUTPUT
                    cout << "-" << blockNamesVec[i] << endl;
                    #endif
                    restructureBlockNamesList.push_back(blockNamesVec[i]);
                } else {
                    displacedBlockNamesList.push_back(blockNamesVec[i]);
                }
            } else {
                displacedBlockNamesList.push_back(blockNamesVec[i]);
            }
        }
    }

    return isUnstableFlag;
}


bool redoDisplacedKOMO(KOMO& komoObject, mlr::KinematicWorld& kinWorld, vector<string>& displacedBlockNamesList) {

    for(string blockName:displacedBlockNamesList) {  // if block has fallen, ensure that its pose is updated
        mlr::KinematicSwitch* kinSwitch = mlr::KinematicSwitch::newSwitch("delete", NULL, blockName.c_str(),
                                                                          kinWorld, 0);

        kinSwitch->apply(kinWorld);
        komoObject.MP->switches.append(kinSwitch);

        mlr::Transformation rel = 0;
        mlr::Transformation finalPose = kinWorld.getBodyByName(blockName.c_str())->X /
                                        kinWorld.getBodyByName("table")->X;
        mlr::KinematicSwitch* newKinSwitch = mlr::KinematicSwitch::newSwitch("rigidZero", "table",
                                                                             blockName.c_str(), kinWorld, 0,
                                                                             rel, finalPose);
        newKinSwitch->apply(kinWorld);
        komoObject.MP->switches.append(newKinSwitch);

        delete kinSwitch;
        delete newKinSwitch;
    }
}


bool restructureKOMO(KOMO& komoObject, mlr::KinematicWorld& kinWorld, mlr::KinematicWorld& finKinWorld, vector<string>& blockNamesList,
                     vector<string>& grabbedBlockNamesList, vector<string>& restructureBlockNamesList) {

    // for any block that falls, ensure that any block on the top of it is also registered as having fallen
    int originalElementsInList = 0;
    do {
        originalElementsInList = restructureBlockNamesList.size();
        for(mlr::Joint* j:kinWorld.joints){
            string blockNameBelow(j->from->name);
            if(!isStringInList(blockNamesList, blockNameBelow)) continue;
            if(isStringInList(restructureBlockNamesList, blockNameBelow)) {
                string blockNameAbove(j->to->name);
                if(!isStringInList(blockNamesList, blockNameAbove)) continue;
                if(isStringInList(grabbedBlockNamesList, blockNameAbove)) continue;
                if(!isStringInList(restructureBlockNamesList, blockNameAbove)) {
                    #if WATCH_OUTPUT
                    cout << "---" << blockNameAbove << endl;
                    #endif
                    restructureBlockNamesList.push_back(blockNameAbove);
                }
            }
        }
    } while(restructureBlockNamesList.size() > originalElementsInList);


    IntervalSet xIntervals;
    xIntervals.add(CENTER_LOC - (0.05 + EPSILON_BLOCKS),
                   CENTER_LOC + (0.05 + EPSILON_BLOCKS));
    xIntervals.add(PLACE_LOC_X - (0.05 + EPSILON_BLOCKS),
                   PLACE_LOC_X + (0.05 + EPSILON_BLOCKS));
    IntervalSet yIntervals;
    yIntervals.add(CENTER_LOC - (0.05 + EPSILON_BLOCKS),
                   CENTER_LOC + (0.05 + EPSILON_BLOCKS));
    yIntervals.add(PLACE_LOC1_Y - (0.05 + EPSILON_BLOCKS),
                   PLACE_LOC1_Y + (0.05 + EPSILON_BLOCKS));
    yIntervals.add(PLACE_LOC2_Y - (0.05 + EPSILON_BLOCKS),
                   PLACE_LOC2_Y + (0.05 + EPSILON_BLOCKS));

    for(mlr::Shape* block:finKinWorld.shapes){
        string blockName(block->name);
        if(!isStringInList(blockNamesList, blockName)) continue;
        xIntervals.add(block->X.pos.x - (block->size[0] + EPSILON_BLOCKS),
                       block->X.pos.x + (block->size[0] + EPSILON_BLOCKS));
        yIntervals.add(block->X.pos.y - (block->size[1] + EPSILON_BLOCKS),
                       block->X.pos.y + (block->size[1] + EPSILON_BLOCKS));
    }

    for(mlr::Shape* block:kinWorld.shapes){
        string blockName(block->name);
        if(!isStringInList(blockNamesList, blockName)) continue;
        if(isStringInList(grabbedBlockNamesList, blockName)) continue;
        if(!isStringInList(restructureBlockNamesList, blockName)) {
            xIntervals.add(block->X.pos.x - (block->size[0] + EPSILON_BLOCKS),
                           block->X.pos.x + (block->size[0] + EPSILON_BLOCKS));
            yIntervals.add(block->X.pos.y - (block->size[1] + EPSILON_BLOCKS),
                           block->X.pos.y + (block->size[1] + EPSILON_BLOCKS));
        }
    }

    for(string blockName:restructureBlockNamesList) {
        mlr::KinematicSwitch* kinSwitch = mlr::KinematicSwitch::newSwitch("delete", NULL, blockName.c_str(),
                                                                          kinWorld, 0);

        kinSwitch->apply(kinWorld);
        komoObject.MP->switches.append(kinSwitch);

        mlr::Body* block = kinWorld.getBodyByName(blockName.c_str());

        float newX = block->X.pos.x;
        float newY = block->X.pos.y;
        int tries = 0;
        while(xIntervals.isInsideIntervals(newX) && yIntervals.isInsideIntervals(newY)) {
            tries += 1;

            if (tries < 500) {
                newX = generateRandomFloat(newX, TABLE_X_LIMIT_MIN, TABLE_X_LIMIT_MAX);
                newY = generateRandomFloat(newY, TABLE_Y_LIMIT_MIN, TABLE_Y_LIMIT_MAX);
            } else {
                newX = getRandomFloatUniform(TABLE_X_LIMIT_MIN, TABLE_X_LIMIT_MAX);
                newY = getRandomFloatUniform(TABLE_Y_LIMIT_MIN, TABLE_Y_LIMIT_MAX);
            }

            if (tries > 1000) {
                mlr::Transformation rel = 0;
                mlr::Transformation finalPose = kinWorld.getBodyByName(blockName.c_str())->X /
                                                kinWorld.getBodyByName("table")->X;
                mlr::KinematicSwitch* newKinSwitch = mlr::KinematicSwitch::newSwitch("rigidZero", "table",
                                                                                     blockName.c_str(), kinWorld, 0,
                                                                                     rel, finalPose);
                newKinSwitch->apply(kinWorld);
                komoObject.MP->switches.append(newKinSwitch);
                return true;
            }
        }

        block->X.pos.x = newX;
        block->X.pos.y = newY;
        block->X.pos.z = TABLE_HEIGHT_BLOCKS;
        block->X.rot.setRpy(0, 0, block->X.rot.getRad());

        double sizeX = kinWorld.getShapeByName(blockName.c_str())->size[0];
        double sizeY = kinWorld.getShapeByName(blockName.c_str())->size[1];
        xIntervals.add(newX - (sizeX + EPSILON_BLOCKS), newX + (sizeX + EPSILON_BLOCKS));
        yIntervals.add(newY - (sizeY + EPSILON_BLOCKS), newY + (sizeY + EPSILON_BLOCKS));

        mlr::Transformation rel = 0;
        mlr::Transformation finalPose = kinWorld.getBodyByName(blockName.c_str())->X /
                                        kinWorld.getBodyByName("table")->X;
        mlr::KinematicSwitch* newKinSwitch = mlr::KinematicSwitch::newSwitch("rigidZero", "table",
                                                                             blockName.c_str(), kinWorld, 0,
                                                                             rel, finalPose);
        newKinSwitch->apply(kinWorld);
        komoObject.MP->switches.append(newKinSwitch);

        delete kinSwitch;
        delete newKinSwitch;
    }

    return false;
}


void resetFinalFilePos(mlr::KinematicWorld& ogFinKinWorld, mlr::KinematicWorld& currFinKinWorld,
                       vector<string>& restructureBlockNamesList) {
    for(string blockName: restructureBlockNamesList) {
        mlr::Body* currBody = currFinKinWorld.getBodyByName(blockName.c_str());
        mlr::Body* ogBody = ogFinKinWorld.getBodyByName(blockName.c_str());

        currBody->X.pos.x = ogBody->X.pos.x;
        currBody->X.pos.y = ogBody->X.pos.y;

        string surfaceName;
        for(mlr::Joint* j:currFinKinWorld.joints){
            string blockNameCurr(j->to->name);
            string blockNameBelow(j->from->name);
            if (blockNameCurr == blockName) {
                surfaceName = blockNameBelow;
                break;
            }
        }

        mlr::Transformation finalPose = currFinKinWorld.getBodyByName(blockName.c_str())->X /
                                        currFinKinWorld.getBodyByName(surfaceName.c_str())->X;

        moveBlockInWorld(currFinKinWorld, blockName.c_str(), surfaceName.c_str(), finalPose);
    }
}


bool runPhysxSimulation(KOMO& komoObject, mlr::KinematicWorld& finKinWorld, mlr::KinematicWorld& ogFinKinWorld,
                        vector<string>& blockNamesList,
                        vector<int>& checkpointTimesList, vector<string>& grabbedBlockNamesList,
                        string outputFilePath, string outputFinalFilePath, bool runSimulation, bool isDiff) {

    bool isUnstable = false;
    int grabbedBNIndexCounter = 0;

    int checkpointIndexCounter = 1;  // ignore the first checkpoint
    for(uint timeT = 1; timeT < komoObject.MP->T; timeT++){

        if(timeT == checkpointTimesList[checkpointIndexCounter]) {  // each checkpoint represents a move of the robot

            mlr::KinematicWorld kinWorldCopy =* komoObject.MP->configurations(timeT + komoObject.MP->k_order);

            vector<string> currGrabbedBlockNamesList;
            vector<string> displacedBlockNamesList;
            vector<string> restructureBlockNamesList;
            if(grabbedBlockNamesList.size() > 0) {
                split(grabbedBlockNamesList[grabbedBNIndexCounter], COMMANDS_SEPARATOR, currGrabbedBlockNamesList);
            }

            if (!runSimulation) {
                printOutGFile(kinWorldCopy, outputFilePath);
            } else {
                isUnstable = simulateWorld(komoObject, kinWorldCopy, finKinWorld, isDiff,
                                           blockNamesList, currGrabbedBlockNamesList,
                                           displacedBlockNamesList, restructureBlockNamesList);

                if (displacedBlockNamesList.size() > 0) redoDisplacedKOMO(komoObject, kinWorldCopy, displacedBlockNamesList);

                if (restructureBlockNamesList.size() > 0) { // only if diff, do restructuring
                    bool flagRetry = false;
                    do {
                        flagRetry = restructureKOMO(komoObject, kinWorldCopy, finKinWorld, blockNamesList, currGrabbedBlockNamesList, restructureBlockNamesList);
                    } while (flagRetry);
                }

                resetFinalFilePos(ogFinKinWorld, finKinWorld, restructureBlockNamesList);
                printOutGFile(kinWorldCopy, outputFilePath);
                printOutGFile(finKinWorld, outputFinalFilePath);
            }

            if (grabbedBNIndexCounter < grabbedBlockNamesList.size() - 1) grabbedBNIndexCounter++;
            if (checkpointIndexCounter < checkpointTimesList.size() - 1) checkpointIndexCounter++;

            if (checkpointIndexCounter >= checkpointTimesList.size() - 1) return isUnstable;  // ignore last checkpoint
        }
    }

    return isUnstable;
}


double computeSumOfSquaredErrors(KOMO& komoObject){
    return komoObject.opt->UCP.get_costs();
}


double computeKineticEnergy(KOMO& komoObject, mlr::KinematicWorld& initKinWorld,
                            vector<string>& blockNamesList, vector<int>& checkpointTimesList){
    double dt = 0.05;
    double totalKineticEnergy = 0;
    double previousKE = 0.;
    double mass = 1.0;

    int index = 0;

    for(mlr::Body* b : initKinWorld.bodies){

        string blockName(b->name);

        if(isStringInList(blockNamesList, blockName)) {

            previousKE = 0.;
//            double initPE = 0.;
//            double finalPE = 0.;

            for(uint timeT = 1; timeT < komoObject.MP->T; timeT++){

                mlr::KinematicWorld* W0 = komoObject.MP->configurations(timeT - 1 + komoObject.MP->k_order);
                mlr::KinematicWorld* W1 = komoObject.MP->configurations(timeT + komoObject.MP->k_order);

                mlr::Body* b0 = W0->getBodyByName(b->name);
                mlr::Body* b1 = W1->getBodyByName(b->name);

//                if (timeT == 1) {
//                    initPE = mass * 9.8 * b0->X.pos.z;
//                }
//
//                if (timeT == komoObject.MP->T - 1) {
//                    finalPE = mass * 9.8 * b1->X.pos.z;
//                }

                double distanceBetweenBlocks = (b1->X.pos - b0->X.pos).length();
                double velocity = distanceBetweenBlocks / dt;
                double kineticEnergy = 0.5 * mass * (velocity * velocity);

                totalKineticEnergy += std::abs(kineticEnergy - previousKE) / dt;
                previousKE = kineticEnergy;
            }

//            totalKineticEnergy += (finalPE - initPE);  // add change in potential energy
        }
    }

    return totalKineticEnergy;
}


int main(int argc,char** argv){
    int argv_index_counter = 1;

    bool runSimulation = false;
    istringstream(argv[argv_index_counter++]) >> runSimulation;

    string baseFilePath = argv[argv_index_counter++];
    string initFilePath = baseFilePath + argv[argv_index_counter++];  // can be an intermediate file path
    string finalFilePath = baseFilePath + argv[argv_index_counter++];
    string ogFinalFilePath = baseFilePath + argv[argv_index_counter++];
    string outputFilePath = baseFilePath + argv[argv_index_counter++];

    string variationString = argv[argv_index_counter++];

    string blockNamesString = argv[argv_index_counter++];  // names of all the blocks in the plan
    string planAsString = argv[argv_index_counter++];
    string grabbedBlockNamesString = argv[argv_index_counter++];

    double noiseXLeft = stod(argv[argv_index_counter++]);
    double noiseYLeft = stod(argv[argv_index_counter++]);
    double noiseXRight = stod(argv[argv_index_counter++]);
    double noiseYRight = stod(argv[argv_index_counter++]);

    bool isDiff = false;
    istringstream(argv[argv_index_counter++]) >> isDiff;

    string outputGFilePath = outputFilePath + "state" + "_" + variationString + ".g";
    string outputKEFilePath = outputFilePath + "ke" + "_" + variationString + ".txt";
    string outputStabilityFilePath = outputFilePath + "stability" + "_" + variationString + ".txt";

    KOMO komoObject;
    komoObject.verbose = 0;

    mlr::KinematicWorld initKinWorld;
    mlr::KinematicWorld finalKinWorld;
    mlr::KinematicWorld ogFinalKinWorld;
    initializeWorkSpace(komoObject, initKinWorld, finalKinWorld, ogFinalKinWorld,
                        initFilePath, finalFilePath, ogFinalFilePath);

    vector<string> blockNamesList;
    split(blockNamesString, PLAN_SEPARATOR, blockNamesList);

    vector<string> grabbedBlockNamesList;
    std::string noneString ("None");
    if (grabbedBlockNamesString.compare(noneString) != 0) {
        split(grabbedBlockNamesString, PLAN_SEPARATOR, grabbedBlockNamesList);
    }

    vector<int> outCheckpointTimesList;
    setupKOMO(komoObject, initKinWorld, finalKinWorld, planAsString,
              noiseXLeft, noiseYLeft, noiseXRight, noiseYRight, outCheckpointTimesList);

    #if WATCH_OUTPUT
    cout << "----------------------" << endl;
    cout << "PLAN:" << planAsString << endl;
    cout << "VAR NUM: " << variationString << endl;
    cout << "NOISE_LEFT_X : " << noiseXLeft << endl;
    cout << "NOISE_LEFT_Y : " << noiseYLeft << endl;
    cout << "NOISE_RIGHT_X: " << noiseXRight << endl;
    cout << "NOISE_RIGHT_Y: " << noiseYRight << endl;
    cout << "Running simulation..." << endl;
    #endif

    bool isUnstable = runPhysxSimulation(komoObject, finalKinWorld, ogFinalKinWorld,
                                         blockNamesList, outCheckpointTimesList, grabbedBlockNamesList,
                                         outputGFilePath, finalFilePath, runSimulation, isDiff);

    float kineticEnergyExpended = computeKineticEnergy(komoObject, initKinWorld,
                                                       blockNamesList, outCheckpointTimesList);

    printOutDataFile(kineticEnergyExpended, isUnstable, outputKEFilePath);

//    float sumOfSquaredErrors = computeSumOfSquaredErrors(komoObject);
//    printOutDataFile(sumOfSquaredErrors, isUnstable, outputKEFilePath);

    return 0;
}

