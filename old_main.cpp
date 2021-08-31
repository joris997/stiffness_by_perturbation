#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <OpenSim/OpenSim.h>

// 15-08: 0.182 - 0.665

//const std::string absPath = "/home/joris997/thesis_perturbation/data";
//const std::string absPath = "/home/none/Downloads/TUd_2020-2021/Research/master_thesis/2_experiments/motion_capture/matlab_tests/leg_stiffness_cpp/data";
const std::string absPath = "/home/none/Downloads/TUd_2020-2021/Research/master_thesis/2_experiments/motion_capture/Guoping/sub06";
const double startTime = 0.188;
const double finalTime = 0.688;
const double addedForce = 100.0;

#include "createFiles.hpp"
#include "compute.hpp"

int main(){
    std::string modelPath = absPath + "/Analysis/RRA/results/RRA-15-06/sub06_model_RRA-15-06.osim";

//    std::string GRFPath = "/home/none/Downloads/TUd_2020-2021/Research/master_thesis/2_experiments/motion_capture/Guoping/sub06/GRFMot/grf-15-08.mot";
    std::string GRFPath = absPath + "/GRFMot/grf-15-06.mot";
    std::string forcePath = absPath + "/perturbation_test/appliedForce.sto";
    std::string statesPath = absPath + "/Analysis/CMC/results/CMC-15-06/sub06_states.sto";
    std::string controlsPath = absPath + "/Analysis/CMC/results/CMC-15-06/sub06_controls.sto";

    // Get starting and final index of the states file
    int startIdx = getStartIdx(statesPath,startTime);
    int finalIdx = getStartIdx(statesPath,finalTime);
    std::cout << "start index: " << startIdx << " (t = " << startTime << ")" << std::endl;
    std::cout << "end index:   " << finalIdx << " (t = " << finalTime << ")" << std::endl;
    std::vector<double> stiffnesses;
    std::vector<double> times;

    for(int i=startIdx; i<finalIdx; i=i+5){
        bool left = true;
        // Load and initialize model
        OpenSim::Model model(modelPath);
        model.initSystem();

        // load in state data and create 'StatesTrajectory' object which can be
        // indexed to extract the correct state.
        OpenSim::Storage storage(statesPath);
        OpenSim::StatesTrajectory stateTrajectory =
                OpenSim::StatesTrajectory::createFromStatesStorage(model,storage,false,false,true);

        // The state of this iteration
        SimTK::State state = stateTrajectory[i];
        double time = state.getTime();
        state.setTime(0.0);
        // Set the model to the correct state
        model.realizeAcceleration(state);

        // get the CoM location of the pelvis, left foot, and right foot
        // expressed in the global frame, state dependend
        // TODO: see 'getXYZ' function declaration
        SimTK::Vec3 pelvisXYZ = getXYZ(model,state,"pelvis");
        SimTK::Vec3 footLXYZ = getXYZ(model,state,"calcn_l");
        SimTK::Vec3 footRXYZ = getXYZ(model,state,"calcn_r");

        // Create forces on the left or right foot in direction of the pelvis
        // and in the direction of the soon to be defined PointOnLine constraint
        std::string footBodyName;
        if(left){
            footBodyName = "calcn_l";
        } else {
            footBodyName = "calcn_r";
        }

        createForceFile(pelvisXYZ,footLXYZ,footRXYZ,
                        left,
                        true,       // add perturbation
                        forcePath,
                        GRFPath,
                        state.getTime());

        // Create control file which is the control sequence of the current
        // iteration copied for the duration of 1 second (activation stays the
        // same for the simulation duration)
        createControlFile(controlsPath,i);
        // Create initial state file, just a file with the state of the current
        // iteration copied once. Used for the ForwardTool
        createStateFile(statesPath,i);

        // Change the direction of the PointOnLineConstraint to properly align
        // with the direction of the foot to pelvis
        OpenSim::PointOnLineConstraint& point_on_line_constraint =
                model.updComponent<OpenSim::PointOnLineConstraint>("/constraintset/PoL");
//        std::string bodySetToFootBody = "/bodyset/" + footBodyName;
//        point_on_line_constraint.setFollowerBodyByName(bodySetToFootBody);

        // Change line_direction_vec, expressed in pelvis frame
        SimTK::Vec3 foot_in_pelvis;
        if (left){
            foot_in_pelvis = pelvisXYZ - footLXYZ;
        } else {
            foot_in_pelvis = pelvisXYZ - footRXYZ;
        }
        point_on_line_constraint.setLineDirection(foot_in_pelvis);
        point_on_line_constraint.set_line_direction_vec(foot_in_pelvis);

        // Set the default value to the required value from the state in this
        // iteration
        std::vector<std::string> coordsToLock = createCoordsToLockVector(left);
        for(unsigned i=0; i<coordsToLock.size(); i++){
            OpenSim::Coordinate& coord =
                    model.updComponent<OpenSim::Coordinate>(coordsToLock[i]);
            coord.set_default_value(valueFromState(coordsToLock[i],true));
            coord.setDefaultValue(valueFromState(coordsToLock[i],true));
            coord.set_locked(true);
            std::cout << "default: " << coord.get_default_value() << std::endl;
        }
        model.finalizeConnections();
        model.finalizeFromProperties();

        // Loop through all coordinates and print those that are 'free'
        std::cout << "\nPRINTING COORDS!" << std::endl;
        for (OpenSim::Coordinate coord : model.updComponentList<OpenSim::Coordinate>()){
            if (!coord.getLocked(state)){
                std::cout << coord.getName() << std::endl;
            }
        }
        std::cout << "END\n" << std::endl;

        // Add controls
        createControlFileXml(controlsPath,i);
        OpenSim::ControlSet controlSet(absPath + "/perturbation_test/_controls.xml");

        OpenSim::ControlSetController controller;
        controller.setControlSet((OpenSim::ControlSet*)OpenSim::Object::SafeCopy(&controlSet));
        model.addController(&controller);

        // Set the model to the proper state again after all the changes
        model.finalizeFromProperties();
        model.print(absPath+"/perturbation_test/model.osim");

        ///////////////////////////
        // CMC FOR STATIC CONFIG //
        ///////////////////////////
        std::string initialStatePath = absPath + "/perturbation_test/state.sto";
//        performCMC(model,GRFPath,initialStatePath,left,state.getTime());

        ////////////////////
        // FORWARD TOOL 1 //
        ////////////////////
        // Create a forward tool and connect the controls file, the initial
        // state file, and the external force file
        OpenSim::ForwardTool fwdTool;
        if (left) {
            fwdTool.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceLeft.xml");
        } else {
            fwdTool.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceRight.xml");
        }
        fwdTool.setModel(model);
        fwdTool.setResultsDir(absPath + "/perturbation_test/");
        fwdTool.setFinalTime(finalTime-startTime);
        fwdTool.setStatesFileName(initialStatePath);

        fwdTool.print(absPath + "/perturbation_test/fwd_setup.xml");

        model.setUseVisualizer(true);
        // Run forward simulation
        fwdTool.run();

        //////////////////
        // ANALYZE TOOL //
        //////////////////
        // Run an AnalyzeTool to get the CoM body positions
        OpenSim::AnalyzeTool anTool(absPath+"/perturbation_test/analyzeTool_setup.xml");
        anTool.setName("analyzeTool");
        anTool.setModelFilename(modelPath);
        anTool.setResultsDir(absPath + "/perturbation_test/");
        anTool.setCoordinatesFileName(absPath + "/perturbation_test/_states.sto");
        anTool.setStartTime(startTime);
        anTool.setFinalTime(finalTime-startTime);
        anTool.run();

        ///////////////////////////
        // STIFFNESS COMPUTATION //
        ///////////////////////////
        std::string positionFile = absPath + "/perturbation_test/analyzeTool_BodyKinematics_pos_global.sto";

        double x0 = getCalcnPelvisDistance(positionFile,left,finalTime/2);
        // and then get it after the additional force
        double xf = getCalcnPelvisDistance(positionFile,left,finalTime);

        // Compute the stiffness
        double stiffness = addedForce/(x0 - xf);
        std::cout << "x0: " << x0 << " xf: " << xf << std::endl;
        std::cout << "stiffness: " << stiffness << std::endl;

        // Add to vectors for printing to file
        stiffnesses.push_back(stiffness);
        times.push_back(time);

        model.disownAllComponents();
    }
    std::ofstream out(absPath + "/perturbation_test/RESULTS.txt");
    out << "time\tstiffness\n";
    for (unsigned i=0; i<times.size(); i++){
        out << times[i] << "\t" << stiffnesses[i] << "\n";
    }
    out.close();
    return 0;
}


//const std::string absPath = "/home/none/Downloads/TUd_2020-2021/Research/master_thesis/2_experiments/motion_capture/matlab_tests/leg_stiffness_cpp/data";
////const std::string absPath = "/home/joris997/thesis_perturbation/data";
//const double finalTime = 1.0;
//const double addedForce = 100.0;

//#include "createFiles.hpp"
//#include "compute.hpp"

//void createConstantStatesFile(const std::string& initialStatePath,
//                              const std::string& constantStatesFile){
//    std::ofstream out(constantStatesFile);
//    std::ifstream in(initialStatePath);

//    int lineno = 0;
//    std::string line;
//    while(std::getline(in,line)){
//        ++lineno;
//        if(lineno < 8){
//            if (lineno == 3){
//                out << "nRows=100\n";
//            } else {
//                out << line << "\n";
//            }
//        }
//        if(lineno == 8){
//            std::string lineCut = line.substr(13,line.length()-1);
//            int numLoops = finalTime*100.0;
//            for(int i=0; i<numLoops+1; i++){
//                out << "      " << std::to_string((double)i/100) << "\t      " << lineCut;
//            }
//            break;
//        }
//    }
//    in.close();
//    out.close();
//}

//void performCMC(OpenSim::Model model, std::string GRFPath,std::string initialStatePath, bool left, SimTK::Real time){
//    SimTK::Vec3 dummyXYZ;
//    std::string forcePath = absPath + "/CMC/appliedForceGRFOnly.sto";
//    // Create a sto file that only contains the GRF
//    createForceFile(dummyXYZ,dummyXYZ,dummyXYZ,
//                    false,      // if left
//                    false,      // add perturbation
//                    forcePath,
//                    GRFPath,
//                    time);

//    // Create a sto file that contains the desired state (constant one)
//    std::string constantStatesFile = absPath + "/CMC/constantStatesFile.sto";
//    createConstantStatesFile(initialStatePath,constantStatesFile);

//    OpenSim::CMCTool cmcTool;
//    cmcTool.setModel(model);
//    cmcTool.setReplaceForceSet(false);
//    cmcTool.setResultsDir(absPath+"/CMC/results/");
//    cmcTool.setDesiredKinematicsFileName(constantStatesFile);

//    cmcTool.setInitialTime(0.0);
//    cmcTool.setFinalTime(1.0);
//    cmcTool.setTimeWindow(0.01);
//    if(left){
//        std::string forceSetPath = absPath + "/CMC/CMC_Actuators_left.xml";
//        OpenSim::Array<std::string> forceSetArray(forceSetPath,1);
//        cmcTool.setForceSetFiles(forceSetArray);
//        cmcTool.setTaskSetFileName(absPath+"/CMC/CMC_Tasks_left.xml");

//        cmcTool.setExternalLoadsFileName(absPath+"/CMC/CMC_Load_left.xml");
//    } else {
//        std::string forceSetPath = absPath + "/CMC/CMC_Actuators_right.xml";
//        OpenSim::Array<std::string> forceSetArray(forceSetPath,1);
//        cmcTool.setForceSetFiles(forceSetArray);
//        cmcTool.setTaskSetFileName(absPath+"/CMC/CMC_Tasks_right.xml");

//        cmcTool.setExternalLoadsFileName(absPath+"/CMC/CMC_Load_right.xml");
//    }
//    cmcTool.run();
//}
//int main(){
//    std::string modelPath = absPath + "/subject5_final.osim";
//    std::string modelPathLeft = absPath + "/subject5_final_left.osim";
//    std::string modelPathRight = absPath + "/subject5_final_right.osim";
//    std::string forcePath = absPath + "/appliedForce.sto";
//    std::string GRFPath = absPath + "/GRFs.mot";
//    std::string statesPath = absPath + "/scaled_states.sto";
//    std::string controlsPath = absPath + "/scaled_controls.sto";

//    // First let's try to get the number of states in the file. We create a
//    // model and statetrajectory also in the for-loop
//    OpenSim::Model modelForN(modelPath);
//    modelForN.initSystem();

//    OpenSim::Storage storageForN(statesPath);
//    OpenSim::StatesTrajectory stateTrajectoryForN =
//            OpenSim::StatesTrajectory::createFromStatesStorage(modelForN,storageForN,false,false,true);

//    int nRows = stateTrajectoryForN.getSize();
//    std::cout << "Detected " << nRows << " states!" << std::endl;

//    std::vector<double> stiffnesses;
//    std::vector<double> times;
//    for(int i=24; i<26; i=i+25){
//        bool left = false;
//        std::string modelPathLoop;
//        if (left){
//            modelPathLoop = modelPathLeft;
//        } else {
//            modelPathLoop = modelPathRight;
//        }

//        // Load and initialize model
//        OpenSim::Model model(modelPathLoop);
//        model.initSystem();

//        // load in state data and create 'StatesTrajectory' object which can be
//        // indexed to extract the correct state.
//        OpenSim::Storage storage(statesPath);
//        OpenSim::StatesTrajectory stateTrajectory =
//                OpenSim::StatesTrajectory::createFromStatesStorage(model,storage,false,false,true);

//        // The state of this iteration
//        SimTK::State state = stateTrajectory[i];
//        double time = state.getTime();
//        state.setTime(0.0);
//        // Set the model to the correct state
//        model.realizeAcceleration(state);

//        // get the CoM location of the pelvis, left foot, and right foot
//        // expressed in the global frame, state dependend
//        // TODO: see 'getXYZ' function declaration
//        SimTK::Vec3 pelvisXYZ = getXYZ(model,state,"pelvis");
//        SimTK::Vec3 footLXYZ = getXYZ(model,state,"calcn_l");
//        SimTK::Vec3 footRXYZ = getXYZ(model,state,"calcn_r");

//        // Create forces on the left or right foot in direction of the pelvis
//        // and in the direction of the soon to be defined PointOnLine constraint
//        std::string footBodyName;
//        if(left){
//            footBodyName = "calcn_l";
//        } else {
//            footBodyName = "calcn_r";
//        }
//        createForceFile(pelvisXYZ,footLXYZ,footRXYZ,
//                        left,
//                        true,       // add perturbation
//                        forcePath,
//                        GRFPath,
//                        state.getTime());

//        // Create control file which is the control sequence of the current
//        // iteration copied for the duration of 1 second (activation stays the
//        // same for the simulation duration)
//        createControlFile(controlsPath,i);
//        // Create initial state file, just a file with the state of the current
//        // iteration copied once. Used for the ForwardTool
//        createStateFile(statesPath,i);

//        // Change the direction of the PointOnLineConstraint to properly align
//        // with the direction of the foot to pelvis
//        OpenSim::PointOnLineConstraint& point_on_line_constraint =
//                model.updComponent<OpenSim::PointOnLineConstraint>("/constraintset/PoL");
////        std::string bodySetToFootBody = "/bodyset/" + footBodyName;
////        point_on_line_constraint.setFollowerBodyByName(bodySetToFootBody);

//        // Change line_direction_vec, expressed in pelvis frame
//        SimTK::Vec3 foot_in_pelvis;
//        if (left){
//            foot_in_pelvis = pelvisXYZ - footLXYZ;
//        } else {
//            foot_in_pelvis = pelvisXYZ - footRXYZ;
//        }
////        SimTK::Vec3 foot_in_pelvis = getXYZinFrame(model,
////                                                   state,
////                                                   footBodyName,
////                                                   "pelvis");
//        point_on_line_constraint.setLineDirection(foot_in_pelvis);
//        point_on_line_constraint.set_line_direction_vec(foot_in_pelvis);

//        // Set the default value to the required value from the state in this
//        // iteration
//        std::vector<std::string> coordsToLock = createCoordsToLockVector(left);
//        for(unsigned i=0; i<coordsToLock.size(); i++){
//            OpenSim::Coordinate& coord =
//                    model.updComponent<OpenSim::Coordinate>(coordsToLock[i]);
//            coord.set_default_value(valueFromState(coordsToLock[i],true));
//            coord.setDefaultValue(valueFromState(coordsToLock[i],true));
//            coord.set_locked(true);
//            std::cout << "default: " << coord.getDefaultValue() << std::endl;
//        }
//        model.finalizeConnections();
//        model.finalizeFromProperties();

//        // Loop through all coordinates and print those that are 'free'
//        std::cout << "\nPRINTING COORDS!" << std::endl;
//        for (OpenSim::Coordinate coord : model.updComponentList<OpenSim::Coordinate>()){
//            if (!coord.getLocked(state)){
//                std::cout << coord.getName() << std::endl;
//            }
//        }
//        std::cout << "END\n" << std::endl;

//        // Add controls
//        createControlFileXml(controlsPath,i);
//        OpenSim::ControlSet controlSet(absPath + "/_controls.xml");

//        OpenSim::ControlSetController controller;
//        controller.setControlSet((OpenSim::ControlSet*)OpenSim::Object::SafeCopy(&controlSet));
//        model.addController(&controller);

//        // Set the model to the proper state again after all the changes
//        model.finalizeFromProperties();
//        model.print(absPath+"/forward_out/model.osim");

//        ///////////////////////////
//        // CMC FOR STATIC CONFIG //
//        ///////////////////////////
//        std::string initialStatePath = absPath + "/state.sto";
//        performCMC(model,GRFPath,initialStatePath,left,state.getTime());

//        //////////////////
//        // FORWARD TOOL //
//        //////////////////
//        // Create a forward tool and connect the controls file, the initial
//        // state file, and the external force file
//        OpenSim::ForwardTool fwdTool;
//        if (left) {
//            fwdTool.setExternalLoadsFileName(absPath + "/externalForceLeft.xml");
//        } else {
//            fwdTool.setExternalLoadsFileName(absPath + "/externalForceRight.xml");
//        }
//        fwdTool.setModel(model);
//        fwdTool.setResultsDir(absPath + "/forward_out/");
//        fwdTool.setFinalTime(finalTime);
//        fwdTool.setStatesFileName(initialStatePath);

//        fwdTool.print("fwd_setup.xml");

//        model.setUseVisualizer(true);
//        // Run forward simulation
//        fwdTool.run();

//        //////////////////
//        // ANALYZE TOOL //
//        //////////////////
//        // Run an AnalyzeTool to get the CoM body positions
//        OpenSim::AnalyzeTool anTool(absPath+"/analyzeTool_setup.xml");
//        anTool.setName("analyzeTool");
//        anTool.setModelFilename(absPath+"/forward_out/model.osim");
//        anTool.setResultsDir(absPath + "/forward_out/");
//        anTool.setCoordinatesFileName(absPath + "/forward_out/_states.sto");
//        anTool.run();

//        ///////////////////////////
//        // STIFFNESS COMPUTATION //
//        ///////////////////////////
//        std::string positionFile = absPath + "/forward_out/analyzeTool_BodyKinematics_pos_global.sto";
//        double x0 = getCalcnPelvisDistance(positionFile,left,finalTime/2);
//        // and then get it after the additional force
//        double xf = getCalcnPelvisDistance(positionFile,left,finalTime);

//        // Compute the stiffness
//        double stiffness = addedForce/(x0 - xf);
//        std::cout << "x0: " << x0 << " xf: " << xf << std::endl;
//        std::cout << "stiffness: " << stiffness << std::endl;

//        // Add to vectors for printing to file
//        stiffnesses.push_back(stiffness);
//        times.push_back(time);

//        model.disownAllComponents();
//    }
//    std::ofstream out(absPath + "/RESULTS.txt");
//    out << "time\tstiffness\n";
//    for (unsigned i=0; i<times.size(); i++){
//        out << times[i] << "\t" << stiffnesses[i] << "\n";
//    }
//    out.close();
//    return 0;
//}
