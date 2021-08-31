#include <OpenSim/OpenSim.h>
#include <string>
#include <iostream>
#include <fstream>
void addPointOnLineConstraint(const std::string& modelPath,
                              const std::string& newModelPath,
                              const std::string& bodyName){
    std::ifstream in(modelPath);
    std::ofstream out(newModelPath);

    int lineno = 0;
    std::string line;
    while(std::getline(in,line)){
        ++lineno;
        if (lineno == 4801){
            out << "<objects>\n";
            out << "<PointOnLineConstraint name=\"PoL\">\n";
            out << "<isDisabled>false</isDisabled>\n";
            out << "<socket_line_body>/bodyset/pelvis</socket_line_body>\n";
            out << "<line_direction_vec>0 0 0</line_direction_vec>\n";
            out << "<point_on_line>0 0 0</point_on_line>\n";
            out << "<socket_follower_body>/bodyset/"+bodyName+"</socket_follower_body>\n";
            out << "<point_on_follower>0 0 0</point_on_follower>\n";
            out << "</PointOnLineConstraint>\n";
            out << "</objects>\n";
        } else {
            out << line + "\n";
        }
    }
    in.close();
    out.close();
}

void createConstantStatesFile(const std::string& initialStatePath,
                              const std::string& constantStatesFile){
    std::ofstream out(constantStatesFile);
    std::ifstream in(initialStatePath);

    int lineno = 0;
    std::string line;
    while(std::getline(in,line)){
        ++lineno;
        if(lineno < 8){
            if (lineno == 3){
                out << "nRows=100\n";
            } else {
                out << line << "\n";
            }
        }
        if(lineno == 8){
            std::string lineCut = line.substr(13,line.length()-1);
            int numLoops = deltaTime*1000.0;
            for(int i=0; i<numLoops+1; i++){
                out << "      " << std::to_string((double)i/100) << "\t      " << lineCut;
            }
            break;
        }
    }
    in.close();
    out.close();
}

void createForceFiles_1_2(SimTK::Vec3 pelvisCoM,
                          SimTK::Vec3 footLCoM,
                          SimTK::Vec3 footRCoM,
                          bool left,
                          const std::string& forcePath1,
                          const std::string& forcePath2,
                          const std::string& GRFPath,
                          SimTK::Real time){
    SimTK::Vec3 footCoM;
    if(left){
        footCoM = footLCoM;
    } else {
        footCoM = footRCoM;
    }
    std::ofstream out(forcePath1);
    out << "name appliedForce.sto\n";
    out << "datacolumns 10\n";
    std::string dataRows = "datarows "+std::to_string((int)(deltaTime)*1000+1)+"\n";
    out << dataRows;
    std::string range = "range 0.000000 "+std::to_string(deltaTime)+"\n";
    out << range;
    out << "endheader\n";
    out << "time\tforce_vx\tforce_vy\tforce_vz\tforce_px\tforce_py\tforce_pz\ttorque_x\ttorque_y\ttorque_z\n";

    // Index all the GRF lines in the GRF file into a vector to later check
    // the index closest to the time argument
    std::vector<std::vector<double>> GRFvalues;
    std::ifstream GRFin(GRFPath);
    int lineno = 0;
    std::string line;
    while (std::getline(GRFin,line)){
        ++lineno;
        std::stringstream instream(line);
        std::string value;
        std::vector<double> valueVector;
        if(lineno>=8){
            while(std::getline(instream,value,'\t')){
                valueVector.push_back(std::stod(value));
            }
            GRFvalues.push_back(valueVector);
        }
    }
    // Now find the closest index. Time is first index of each vector
    double timeDouble = (double)time;
    int idx = 0;
    for(int i=0; i<GRFvalues.size(); i++){
        if(timeDouble - GRFvalues[i][0] < 0.0){
            idx = i;
            break;
        }
    }
    // Assign the GRF values from the GRF file to double variables. Later to be
    // added with the additional force
    double fx, fy, fz;
    if (left){
        fx = GRFvalues[idx][1];
        fy = GRFvalues[idx][2];
        fz = GRFvalues[idx][3];
    } else {
        fx = GRFvalues[idx][10];
        fy = GRFvalues[idx][11];
        fz = GRFvalues[idx][12];
    }

    int numLoops = deltaTime*1000.0;
    for(int i=0; i<numLoops; i++){            // from 0.0 to half
        out << std::to_string((double)i/1000)+"\t"+std::to_string(fx)+"\t"+
               std::to_string(fy)+"\t"+std::to_string(fz)+"\t"+
               "0.000\t0.000\t0.000\t0.000\t0.000\t0.000\n";
    }
    out.close();

    // NOW WITH THE ADDED FORCE
    std::ofstream out2(forcePath2);
    out2 << "name appliedForce.sto\n";
    out2 << "datacolumns 10\n";
    out2 << dataRows;
    out2 << range;
    out2 << "endheader\n";
    out2 << "time\tforce_vx\tforce_vy\tforce_vz\tforce_px\tforce_py\tforce_pz\ttorque_x\ttorque_y\ttorque_z\n";

    double fxA, fyA, fzA, alpha, beta, gamma, F;
    F = addedForce;
    // Get the foot to pelvis vector for the direction of the additional force
    SimTK::Vec3 footToPelvis = pelvisCoM - footCoM;
    // Magnitude of the vector to normalize the additional force
    double footToPelvisMagnitude = footToPelvis.norm();

    // Angles for x, y, and z axis, not used
    alpha = acos(footToPelvis[0]/footToPelvisMagnitude);
    beta = acos(footToPelvis[1]/footToPelvisMagnitude);
    gamma = acos(footToPelvis[2]/footToPelvisMagnitude);

    // Additional force in the x, y, and z direction
    fxA = F*footToPelvis[0]/footToPelvisMagnitude;
    fyA = F*footToPelvis[1]/footToPelvisMagnitude;
    fzA = F*footToPelvis[2]/footToPelvisMagnitude;

    // Total force and force applied in the beginning to get to equilibrium
    // point. Then we apply the total force in the second half
    if(left){
        fx = fxA + GRFvalues[idx][1];
        fy = fyA + GRFvalues[idx][2];
        fz = fzA + GRFvalues[idx][3];
    } else {
        fx = fxA + GRFvalues[idx][10];
        fy = fyA + GRFvalues[idx][11];
        fz = fzA + GRFvalues[idx][12];
    }

    // Write the forces to file
    for(int i=0; i<numLoops; i++){
        out2 << std::to_string((double)i/1000)+"\t"+std::to_string(fx)+"\t"+
                std::to_string(fy)+"\t"+std::to_string(fz)+"\t"+
                "0.000\t0.000\t0.000\t0.000\t0.000\t0.000\n";
    }
    GRFin.close();
    out2.close();
}
void createForceFile(SimTK::Vec3 pelvisCoM,
                     SimTK::Vec3 footLCoM,
                     SimTK::Vec3 footRCoM,
                     bool left,
                     bool addPerturbation,
                     const std::string& forcePath,
                     const std::string& GRFPath,
                     SimTK::Real time){
    // Function to create a forc file that indicates the foot applied to the
    // force. Representing the GRF + an excitation term used to compute the
    // stiffness and/or damping
    // IN: pelvisCoM:   pelvis XYZ vector expressed in the global frame
    //     footLCoM:    left foot XYZ vector '' ''
    //     footRCoM:    right foot XYZ vector '' ''
    //     left:        boolean indicating left or right foot
    //     forcePath:   path string of the to-print file
    //     GRFPath:     path string pointing to the GRF file
    //     time:        time of the current it. Should find GRFs at this time
    SimTK::Vec3 footCoM;
    if(left){
        footCoM = footLCoM;
    } else {
        footCoM = footRCoM;
    }
    std::ofstream out(forcePath);
    out << "name appliedForce.sto\n";
    out << "datacolumns 10\n";
    std::string dataRows = "datarows "+std::to_string((int)(deltaTime)*1000+1)+"\n";
    out << dataRows;
    std::string range = "range 0.000000 "+std::to_string(deltaTime)+"\n";
    out << range;
    out << "endheader\n";
    out << "time\tforce_vx\tforce_vy\tforce_vz\tforce_px\tforce_py\tforce_pz\ttorque_x\ttorque_y\ttorque_z\n";

    // Index all the GRF lines in the GRF file into a vector to later check
    // the index closest to the time argument
    std::vector<std::vector<double>> GRFvalues;
    std::ifstream GRFin(GRFPath);
    int lineno = 0;
    std::string line;
    std::cout << "asdf" << std::endl;
    while (std::getline(GRFin,line)){
        ++lineno;
//        std::cout << line << std::endl;
        std::stringstream instream(line);
        std::string value;
        std::vector<double> valueVector;
        if(lineno>=8){
            while(std::getline(instream,value,'\t')){
                valueVector.push_back(std::stod(value));
            }
            GRFvalues.push_back(valueVector);
        }
    }
    // Now find the closest index. Time is first index of each vector
    double timeDouble = (double)time;
    int idx = 0;
    for(int i=0; i<GRFvalues.size(); i++){
        if(timeDouble - GRFvalues[i][0] < 0.0){
            idx = i;
            break;
        }
    }
    // Assign the GRF values from the GRF file to double variables. Later to be
    // added with the additional force
    double fxL, fyL, fzL, fxR, fyR, fzR;
    fxL = GRFvalues[idx][1]; fyL = GRFvalues[idx][2]; fzL = GRFvalues[idx][3];
    fxR = GRFvalues[idx][10]; fyR = GRFvalues[idx][11]; fzR = GRFvalues[idx][12];

    // Additional force
    double fxA, fyA, fzA, alpha, beta, gamma, F;
    F = addedForce;
    // Get the foot to pelvis vector for the direction of the additional force
    SimTK::Vec3 footToPelvis = pelvisCoM - footCoM;
    // Magnitude of the vector to normalize the additional force
    double footToPelvisMagnitude = footToPelvis.norm();

    // Angles for x, y, and z axis, not used
    alpha = acos(footToPelvis[0]/footToPelvisMagnitude);
    beta = acos(footToPelvis[1]/footToPelvisMagnitude);
    gamma = acos(footToPelvis[2]/footToPelvisMagnitude);

    // Additional force in the x, y, and z direction
    fxA = F*footToPelvis[0]/footToPelvisMagnitude;
    fyA = F*footToPelvis[1]/footToPelvisMagnitude;
    fzA = F*footToPelvis[2]/footToPelvisMagnitude;

    // Total force and force applied in the beginning to get to equilibrium
    // point. Then we apply the total force in the second half
    double fx, fy, fz, fx0, fy0, fz0;
    if(addPerturbation){
        if(left){
            fx0 = fxL; fy0 = fyL; fz0 = fzL;
            fx = fxA + fxL;
            fy = fyA + fyL;
            fz = fzA + fzL;
        } else {
            fx0 = fxR; fy0 = fyR; fz0 = fzR;
            fx = fxA + fxR;
            fy = fyA + fyR;
            fz = fzA + fzR;
        }
    } else {
        fx0 = fxL; fy0 = fyL; fz0 = fzL;
        fx = fxL; fy = fyL; fz = fzL;
    }

    // Write the forces to file
    int numLoops = deltaTime*1000.0;
    for(int i=0; i<numLoops/2; i++){            // from 0.0 to half
        out << std::to_string((double)i/1000)+"\t"+std::to_string(fx0)+"\t"+
               std::to_string(fy0)+"\t"+std::to_string(fz0)+"\t"+
               "0.000\t0.000\t0.000\t0.000\t0.000\t0.000\n";
    }
    for(int i=numLoops/2; i<numLoops+1; i++){   // from half to end
        out << std::to_string((double)i/1000)+"\t"+std::to_string(fx)+"\t"+
               std::to_string(fy)+"\t"+std::to_string(fz)+"\t"+
               "0.000\t0.000\t0.000\t0.000\t0.000\t0.000\n";
    }
    GRFin.close();
    out.close();
}

void createControlFileXml(const std::string& oldControlFile, const int idx){
    std::ifstream in(oldControlFile);

    std::vector<std::string> actuators;
    std::vector<double> actuatorValues;

    std::string line;
    int lineno = 0;
    while(std::getline(in,line)){
        ++lineno;
        std::stringstream stringStream(line);
        std::string value;
        // Fill array of all actuator names
        if (lineno == 7){
            int valueno = 0;
            while(std::getline(stringStream,value,'\t')){
                ++valueno;
                if (valueno > 1){
                    actuators.push_back(value);
                }
            }
        }
        // Get the corresponding vector of values
        if (lineno == idx+8){
            int valueno = 0;
            while(std::getline(stringStream,value,'\t')){
                ++valueno;
                if (valueno > 1){
//                    std::cout << "val: " << value << std::endl;
                    actuatorValues.push_back(std::stod(value));
                }
            }
            break;
        }
    }
    // Now create a controls.xml file (as FWD tool does not work with .sto)
    OpenSim::ControlSet cs;
    for (unsigned i=0; i<actuators.size(); i++){
        OpenSim::ControlLinear cl;
        cl.setName(actuators[i]+".excitation");
        cl.setIsModelControl(true);
        cl.setExtrapolate(true);
        cl.setDefaultParameterMin(0.02);
        cl.setDefaultParameterMax(1);
        cl.setFilterOn(false);
        cl.setUseSteps(false);
        cl.setKp(100);
        cl.setKv(20);
        // Create nodes
        OpenSim::ControlLinearNode cln1;
        cln1.setTime(0.0);
        cln1.setValue(actuatorValues[i]);
        OpenSim::ControlLinearNode cln2;
        cln2.setTime(deltaTime);
        cln2.setValue(actuatorValues[i]);
        // Add nodes to ControlLinear
        cl.insertNewValueNode(0,cln1);
        cl.insertNewValueNode(1,cln2);
        // Add ControlLinear to ControlSet
        cs.insert(i,cl);
    }
    cs.print(absPath + "/perturbation_test/_controls.xml");
}

void createControlFile(const std::string& oldControlFile, const int idx){
    // Function to create a control file which indicates the activation of the
    // muscles for all time of the forward simulation
    // IN: oldControlFile:  path string where the CMC control file is located
    //     idx:             idx that corresponds to current iteration considered
    std::string newControlFile = absPath + "controls.sto";
    std::ifstream in(oldControlFile);
    std::ofstream out(newControlFile);

    out << "controls\n";
    out << "version=1\n";
    out << "nRows="+std::to_string((int)deltaTime*1000+1)+"\n";
    out << "nColumns=114\n";
    out << "inDegrees=no\n";
    out << "endheader\n";

    int lineno = 0;
    std::string line;
    std::string lineCut;
    while(std::getline(in,line)){
        ++lineno;
        // Header of the variables
        if(lineno==7){
            out << line+"\n";
        }
        // 8 non-value lines (begin + header) so idx+8 is the row of the idx
        // under consideration
        if(lineno==idx+8){
            lineCut = line.substr(16,line.length()-1);
            for(int i=0; i<(int)deltaTime*1000+1; i++){ // to 101 to have 0.00 to 1.00
                out << "      " + std::to_string(i/(deltaTime*1000)) + lineCut+"\n";
            }
            break;
        }
    }
    in.close();
    out.close();
}

void createStateFile(const std::string& statesFile, const int idx){
    // Function to create an initial states file. A single column file with all
    // the values of the coordinates of the model.
    // IN: statesFile:  string path pointing to where the state file from the
    //                  CMC can be found
    //     idx:         idx that corresponds to current iteration considered
    std::string stateFile = absPath + "/perturbation_test/state.sto";
    std::ifstream in(statesFile);
    std::ofstream out(stateFile);

    int lineno = 0;
    std::string line;
    std::string lineCut;
    while(std::getline(in,line)){
        ++lineno;
        // Just copy the start of the state file + header of the variables.
        if(lineno < 3 || (lineno > 3 && lineno < 8)){
            out << line+"\n";
        }
        if(lineno==3){
            out << "nRows=1\n";
        }
        // 8 non-value lines (begin + header) so idx+8 is the row of the idx
        // under consideration
        if(lineno==idx+8){
            lineCut = line.substr(17,line.length()-1);
            out << "0.0000\t" + lineCut+"\n";
            break;
        }
    }
    in.close();
    out.close();

    std::ifstream in2(stateFile);
    lineno = 0;
    int id = 0;
    int id2 = 0;
    while(std::getline(in2,line)){
        ++lineno;
        std::stringstream instream(line);
        std::string value;
        if(lineno==7){
            while(std::getline(instream,value,'\t')){
                if (value.compare("/forceset/addlong_r/fiber_length") == 0){
                    std::cout << "FOUND: " << value << std::endl;
                    break;
                }
                id++;
            }
        }
        if(lineno==8){
            while(std::getline(instream,value,'\t')){
                if(id == id2){
                    std::cout << "VALUE IS: " << value << std::endl;
                    break;
                }
                id2++;
            }
        }
    }
    in2.close();
}


std::vector<std::string> createCoordsToLockVector(bool left){
    std::vector<std::string> coordsToLock;
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tilt");
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_list");
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_rotation");
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tx");
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_ty");
    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tz");

    coordsToLock.push_back("/jointset/back/lumbar_extension");
    coordsToLock.push_back("/jointset/back/lumbar_bending");
    coordsToLock.push_back("/jointset/back/lumbar_rotation");

    coordsToLock.push_back("/jointset/acromial_r/arm_flex_r");
    coordsToLock.push_back("/jointset/acromial_r/arm_add_r");
    coordsToLock.push_back("/jointset/acromial_r/arm_rot_r");
    coordsToLock.push_back("/jointset/elbow_r/elbow_flex_r");
    coordsToLock.push_back("/jointset/radioulnar_r/pro_sup_r");
    coordsToLock.push_back("/jointset/radius_hand_r/wrist_flex_r");
    coordsToLock.push_back("/jointset/radius_hand_r/wrist_dev_r");

    coordsToLock.push_back("/jointset/acromial_l/arm_flex_l");
    coordsToLock.push_back("/jointset/acromial_l/arm_add_l");
    coordsToLock.push_back("/jointset/acromial_l/arm_rot_l");
    coordsToLock.push_back("/jointset/elbow_l/elbow_flex_l");
    coordsToLock.push_back("/jointset/radioulnar_l/pro_sup_l");
    coordsToLock.push_back("/jointset/radius_hand_l/wrist_flex_l");
    coordsToLock.push_back("/jointset/radius_hand_l/wrist_dev_l");
    if (left){
        coordsToLock.push_back("/jointset/hip_r/hip_flexion_r");
        coordsToLock.push_back("/jointset/hip_r/hip_adduction_r");
        coordsToLock.push_back("/jointset/hip_r/hip_rotation_r");
        coordsToLock.push_back("/jointset/knee_r/knee_angle_r");
        coordsToLock.push_back("/jointset/ankle_r/ankle_angle_r");
        coordsToLock.push_back("/jointset/subtalar_r/subtalar_angle_r");
        coordsToLock.push_back("/jointset/mtp_r/mtp_angle_r");
    } else {
        coordsToLock.push_back("/jointset/hip_l/hip_flexion_l");
        coordsToLock.push_back("/jointset/hip_l/hip_adduction_l");
        coordsToLock.push_back("/jointset/hip_l/hip_rotation_l");
        coordsToLock.push_back("/jointset/knee_l/knee_angle_l");
        coordsToLock.push_back("/jointset/ankle_l/ankle_angle_l");
        coordsToLock.push_back("/jointset/subtalar_l/subtalar_angle_l");
        coordsToLock.push_back("/jointset/mtp_l/mtp_angle_l");
    }
    return coordsToLock;
}


//std::vector<std::string> createCoordsToLockVector(bool left){
//    std::vector<std::string> coordsToLock;
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tilt");
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_list");
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_rotation");
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tx");
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_ty");
//    coordsToLock.push_back("/jointset/ground_pelvis/pelvis_tz");

//    coordsToLock.push_back("/jointset/back/lumbar_extension");
//    coordsToLock.push_back("/jointset/back/lumbar_bending");
//    coordsToLock.push_back("/jointset/back/lumbar_rotation");

//    coordsToLock.push_back("/jointset/acromial_r/arm_flex_r");
//    coordsToLock.push_back("/jointset/acromial_r/arm_add_r");
//    coordsToLock.push_back("/jointset/acromial_r/arm_rot_r");
//    coordsToLock.push_back("/jointset/elbow_r/elbow_flex_r");
//    coordsToLock.push_back("/jointset/radioulnar_r/pro_sup_r");
//    coordsToLock.push_back("/jointset/radius_hand_r/wrist_flex_r");
//    coordsToLock.push_back("/jointset/radius_hand_r/wrist_dev_r");

//    coordsToLock.push_back("/jointset/acromial_l/arm_flex_l");
//    coordsToLock.push_back("/jointset/acromial_l/arm_add_l");
//    coordsToLock.push_back("/jointset/acromial_l/arm_rot_l");
//    coordsToLock.push_back("/jointset/elbow_l/elbow_flex_l");
//    coordsToLock.push_back("/jointset/radioulnar_l/pro_sup_l");
//    coordsToLock.push_back("/jointset/radius_hand_l/wrist_flex_l");
//    coordsToLock.push_back("/jointset/radius_hand_l/wrist_dev_l");
//    if (left){
//        coordsToLock.push_back("/jointset/hip_r/hip_flexion_r");
//        coordsToLock.push_back("/jointset/hip_r/hip_adduction_r");
//        coordsToLock.push_back("/jointset/hip_r/hip_rotation_r");
//        coordsToLock.push_back("/jointset/walker_knee_r/knee_angle_r");
//        coordsToLock.push_back("/jointset/patellofemoral_r/knee_angle_r_beta");
//        coordsToLock.push_back("/jointset/ankle_r/ankle_angle_r");
//        coordsToLock.push_back("/jointset/subtalar_r/subtalar_angle_r");
//        coordsToLock.push_back("/jointset/mtp_r/mtp_angle_r");
//    } else {
//        coordsToLock.push_back("/jointset/hip_l/hip_flexion_l");
//        coordsToLock.push_back("/jointset/hip_l/hip_adduction_l");
//        coordsToLock.push_back("/jointset/hip_l/hip_rotation_l");
//        coordsToLock.push_back("/jointset/walker_knee_l/knee_angle_l");
//        coordsToLock.push_back("/jointset/patellofemoral_l/knee_angle_l_beta");
//        coordsToLock.push_back("/jointset/ankle_l/ankle_angle_l");
//        coordsToLock.push_back("/jointset/subtalar_l/subtalar_angle_l");
//        coordsToLock.push_back("/jointset/mtp_l/mtp_angle_l");
//    }
//    return coordsToLock;
//}
