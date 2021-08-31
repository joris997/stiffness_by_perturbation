#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <OpenSim/OpenSim.h>

const std::string absPath = "/home/none/Downloads/TUd_2020-2021/Research/master_thesis/2_experiments/motion_capture/Guoping/sub06";

const double addedForce = 100.0;
const double deltaTime = 0.01;

#include "createFiles.hpp"
#include "compute.hpp"

int main(){
    std::string modelPath = absPath + "/Analysis/RRA/results/RRA-01-01/sub06_model_RRA-01-01.osim";
    std::string newModelPath = absPath + "/perturbation_test/model.osim";

    std::string GRFPath = absPath + "/GRFMot/grf-01-01.mot";
    std::string forcePath1 = absPath + "/perturbation_test/appliedForce1.sto";
    std::string forcePath2 = absPath + "/perturbation_test/appliedForce2.sto";
    std::string statesPath = absPath + "/Analysis/CMC/results/CMC-01-01/sub06_states.sto";
    std::string controlsPath = absPath + "/Analysis/CMC/results/CMC-01-01/sub06_controls.sto";

    // Get starting and final index of the states file
    int startIdx = getStartIdx(statesPath,0.0);
    int finalIdx = getStartIdx(statesPath,100.0);
    std::cout << "start index: " << startIdx << std::endl;
    std::cout << "end index:   " << finalIdx << std::endl;
    std::vector<double> stiffnesses;
    std::vector<double> times;

    //////////////////////////////////////
    // CREATE MODEL WITH POL CONSTRAINT //
    //////////////////////////////////////
    bool left = true;
    std::string footBodyName;
    if(left){
        footBodyName = "calcn_l";
    } else {
        footBodyName = "calcn_r";
    }
    addPointOnLineConstraint(modelPath,newModelPath,footBodyName);

    // Load and initialize model
    OpenSim::Model model2(newModelPath);
    model2.initSystem();

    // load in state data and create 'StatesTrajectory' object which can be
    // indexed to extract the correct state.
    OpenSim::Storage storage(statesPath);
    OpenSim::StatesTrajectory stateTrajectory =
            OpenSim::StatesTrajectory::createFromStatesStorage(model2,storage,false,false,true);

    for(int i=startIdx; i<finalIdx; i=i+2){
        // Load and initialize model
        OpenSim::Model model(newModelPath);
        model.initSystem();

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
        createForceFiles_1_2(pelvisXYZ,footLXYZ,footRXYZ,
                             left,
                             forcePath1,
                             forcePath2,
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
        std::string bodySetToFootBody = "/bodyset/" + footBodyName;
        point_on_line_constraint.setFollowerBodyByName(bodySetToFootBody);

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

        ////////////////////
        // FORWARD TOOL 1 //
        ////////////////////
        // Create a forward tool and connect the controls file, the initial
        // state file, and the external force file
        OpenSim::ForwardTool fwdTool1;
        if (left) {
            fwdTool1.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceLeft1.xml");
        } else {
            fwdTool1.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceRight1.xml");
        }
        fwdTool1.setModel(model);
        fwdTool1.setResultsDir(absPath + "/perturbation_test/1/");
        fwdTool1.setFinalTime(deltaTime);
        fwdTool1.setStatesFileName(initialStatePath);

        model.setUseVisualizer(true);
        fwdTool1.run();

        ////////////////////
        // FORWARD TOOL 2 //
        ////////////////////
        // Create a forward tool and connect the controls file, the initial
        // state file, and the external force file
        OpenSim::ForwardTool fwdTool2;
        if (left) {
            fwdTool2.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceLeft2.xml");
        } else {
            fwdTool2.setExternalLoadsFileName(absPath + "/perturbation_test/externalForceRight2.xml");
        }
        fwdTool2.setModel(model);
        fwdTool2.setResultsDir(absPath + "/perturbation_test/2/");
        fwdTool2.setFinalTime(deltaTime);
        fwdTool2.setStatesFileName(initialStatePath);

        model.setUseVisualizer(true);
        fwdTool2.run();

        ////////////////////
        // ANALYZE TOOL 1 //
        ////////////////////
        // Run an AnalyzeTool to get the CoM body positions
        OpenSim::AnalyzeTool anTool1(absPath+"/perturbation_test/analyzeTool_setup.xml");
        anTool1.setName("analyzeTool");
        anTool1.setModelFilename(newModelPath);
        anTool1.setResultsDir(absPath + "/perturbation_test/1/");
        anTool1.setCoordinatesFileName(absPath + "/perturbation_test/1/_states.sto");
        anTool1.setStartTime(0.0);
        anTool1.setFinalTime(fwdTool1.getFinalTime());
        anTool1.run();

        ////////////////////
        // ANALYZE TOOL 2 //
        ////////////////////
        // Run an AnalyzeTool to get the CoM body positions
        OpenSim::AnalyzeTool anTool2(absPath+"/perturbation_test/analyzeTool_setup.xml");
        anTool2.setName("analyzeTool");
        anTool2.setModelFilename(newModelPath);
        anTool2.setResultsDir(absPath + "/perturbation_test/2/");
        anTool2.setCoordinatesFileName(absPath + "/perturbation_test/2/_states.sto");
        anTool2.setStartTime(0.0);
        anTool2.setFinalTime(fwdTool2.getFinalTime());
        anTool2.run();

        ///////////////////////////
        // STIFFNESS COMPUTATION //
        ///////////////////////////
        std::string positionFile1 = absPath + "/perturbation_test/1/analyzeTool_BodyKinematics_pos_global.sto";
        std::string positionFile2 = absPath + "/perturbation_test/2/analyzeTool_BodyKinematics_pos_global.sto";
        std::string velocityFile1 = absPath + "/perturbation_test/1/analyzeTool_BodyKinematics_vel_global.sto";
        std::string velocityFile2 = absPath + "/perturbation_test/2/analyzeTool_BodyKinematics_vel_global.sto";
        std::string accelerationFile1 = absPath + "/perturbation_test/1/analyzeTool_BodyKinematics_acc_global.sto";
        std::string accelerationFile2 = absPath + "/perturbation_test/2/analyzeTool_BodyKinematics_acc_global.sto";

        double atTime = deltaTime;
        double x1 = getCalcnPelvisDistance(positionFile1,left,atTime);
        double x2 = getCalcnPelvisDistance(positionFile2,left,atTime);
        double v1 = getCalcnPelvisDistance(velocityFile1,left,atTime);
        double v2 = getCalcnPelvisDistance(velocityFile2,left,atTime);
        double a1 = getCalcnPelvisDistance(accelerationFile1,left,atTime);
        double a2 = getCalcnPelvisDistance(accelerationFile2,left,atTime);
        std::cout << "x1: " << x1 << " x2: " << x2 << std::endl;
        std::cout << "v1: " << v1 << " v2: " << v2 << std::endl;
        std::cout << "a1: " << a1 << " a2: " << a2 << std::endl;

        // Compute the stiffness
        double stiffness;
//        if (x1 > x2){
//            stiffness = addedForce/(v1*deltaTime-v2*deltaTime);
//        } else {
//            stiffness = addedForce/(v2*deltaTime-v1*deltaTime);
//        }
        stiffness = addedForce/(std::abs(x1-x2));
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

