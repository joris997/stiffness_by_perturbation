#include <OpenSim/OpenSim.h>
#include <string>
#include <iostream>
#include <fstream>

int getStartIdx(std::string statesPath, double time){
    std::ifstream in(statesPath);

    int lineno = 0;
    std::string line;
    while(std::getline(in,line)){
        ++lineno;
        if (lineno > 7){
            std::stringstream stringStream(line);
            std::string value;
            std::getline(stringStream,value,'\t');

            if (std::stod(value) > time){
                return lineno - 7;
            }
        }
    }
    return lineno - 7;
}


double getCalcnPelvisDistance(std::string posGlobalFile, bool left, double time){
    // Function to obtain the distance between the Calcn_l and the pelvis.
    SimTK::Vec3 indPelvis, indCalcn_l, indCalcn_r;
    SimTK::Vec3 posPelvis, posCalcn_l, posCalcn_r;

    std::ifstream in(posGlobalFile);
    std::string line;
    int lineno = 0;

    while(std::getline(in,line)){
        ++lineno;
        std::stringstream strStream(line);
        std::string value;
        // Headers
        if (lineno == 19){
            int valueno = 0;
            while(std::getline(strStream,value,'\t')){
                ++valueno;
                if (value.compare("pelvis_X") == 0){
                    indPelvis[0] = valueno;
                }
                if (value.compare("pelvis_Y") == 0){
                    indPelvis[1] = valueno;
                }
                if (value.compare("pelvis_Z") == 0){
                    indPelvis[2] = valueno;
                }

                if (value.compare("calcn_r_X") == 0){
                    indCalcn_r[0] = valueno;
                }
                if (value.compare("calcn_r_Y") == 0){
                    indCalcn_r[1] = valueno;
                }
                if (value.compare("calcn_r_Z") == 0){
                    indCalcn_r[2] = valueno;
                }

                if (value.compare("calcn_l_X") == 0){
                    indCalcn_l[0] = valueno;
                }
                if (value.compare("calcn_l_Y") == 0){
                    indCalcn_l[1] = valueno;
                }
                if (value.compare("calcn_l_Z") == 0){
                    indCalcn_l[2] = valueno;
                }
            }
//            std::cout << indPelvis << std::endl;
//            std::cout << indCalcn_r << std::endl;
//            std::cout << indCalcn_l << std::endl;
        }
    }
    in.close();

    // Get the row where we need to look
    std::ifstream in2(posGlobalFile);
    lineno = 0;

    while(std::getline(in2,line)){
        ++lineno;
        std::stringstream strStream(line);
        std::string value;
        bool flag = false;
        // Headers
        if (lineno > 19){
            while(std::getline(strStream,value,'\t')){
                if (std::stod(line) >= time){
                    flag = true;
                } else {
                    break;
                }
            }
            if (flag){
                break;
            }
        }
    }
    in2.close();
    int linenoCheck = lineno;
//    std::cout << "lineno: " << linenoCheck << std::endl;

    // Get values corresponding to headers
    std::ifstream in3(posGlobalFile);
    lineno = 0;

    while(std::getline(in3,line)){
        ++lineno;
        std::stringstream strStream(line);
        std::string value;
        // Values corresponding to headers
        if (lineno == linenoCheck){
            int valueno = 0;
            while(std::getline(strStream,value,'\t')){
                ++valueno;
                if (valueno == indPelvis[0]){
                    posPelvis[0] = std::stod(value);
                }
                if (valueno == indPelvis[1]){
                    posPelvis[1] = std::stod(value);
                }
                if (valueno == indPelvis[2]){
                    posPelvis[2] = std::stod(value);
                }

                if (valueno == indCalcn_l[0]){
                    posCalcn_l[0] = std::stod(value);
                }
                if (valueno == indCalcn_l[1]){
                    posCalcn_l[1] = std::stod(value);
                }
                if (valueno == indCalcn_l[2]){
                    posCalcn_l[2] = std::stod(value);
                }

                if (valueno == indCalcn_r[0]){
                    posCalcn_r[0] = std::stod(value);
                }
                if (valueno == indCalcn_r[1]){
                    posCalcn_r[1] = std::stod(value);
                }
                if (valueno == indCalcn_r[2]){
                    posCalcn_r[2] = std::stod(value);
                }
            }
//            std::cout << posPelvis << std::endl;
//            std::cout << posCalcn_r << std::endl;
//            std::cout << posCalcn_l << std::endl;
        }
    }
    if (left){
        return (posPelvis-posCalcn_l).norm();
    } else {
        return (posPelvis-posCalcn_r).norm();
    }
}



SimTK::Vec3 getXYZ(OpenSim::Model& model,
                   SimTK::State& st,
                   const std::string& bodyName){
    // Function to get the CoM location of a body in the global reference frame
    // IN: model:       OpenSim model under consideration
    //     st:          state of the model under consideration
    //     bodyName:    string pointing to the body in the osim file
    //                  e.g. "pelvis"
    // OUT: com_ground: size-3 vector of the x, y, and z location of the body
    const OpenSim::Body& body = model.getBodySet().get(bodyName);
    std::cout << body.getMassCenter() << std::endl;
    SimTK::Vec3 com_ground = body.findStationLocationInGround(
                st,body.getMassCenter());

    std::cout << bodyName << ": " << com_ground << std::endl;
    return com_ground;
}

SimTK::Vec3 getXYZinFrame(OpenSim::Model& model,
                          SimTK::State& st,
                          const std::string& pointBodyName,
                          const std::string& baseBodyName){
    // Function to get the CoM location of a body in another body's reference
    // frame
    // IN: model:           OpenSim model under consideration
    //     st:              state of the model under consideration
    //     pointBodyName:   string pointing to the body in the osim file
    //                      e.g. "calcn_r"
    //     baseBodyName:    string pointing to the body that represents the
    //                      frame in which we want to express the point
    // OUT: pointInFrame:   size-3 vector of the x, y, and z location of the
    //                      pointBody in the baseBody frame
    const OpenSim::BodySet& bs = model.getBodySet();
    const OpenSim::Body& pointBody = bs.get(pointBodyName);
    SimTK::Vec3 pointInFrame = pointBody.findStationLocationInAnotherFrame(
                st,
                pointBody.getMassCenter(),
                bs.get(baseBodyName));

    std::cout << pointBodyName << " in " << baseBodyName << ": " <<
                 pointInFrame << std::endl;
    return pointInFrame;
}

double valueFromState(const std::string& coordName,
                      bool position,
                      bool verbose = false){
    // Function to get a 'value' or 'speed' of a coordinate from the pre-made
    // initial state file
    // IN: coordName:   String path pointing to the coordinate in the .osim
    //                  file. e.g. /jointset/pelvis_ty
    //     position:    boolean indicating whether we are searching for a
    //                  /value (position) or a /speed (velocity)
    //     verbose:     optional argument indicating whether to print
    //                  intermediate results
    // OUT: value:      double indicating the value or speed of that coordinate
    std::ifstream in(absPath + "state.sto");
    std::string valueOrSpeed;
    // add '/value' or '/speed' depending on the 'position' input argument
    if(position){
        valueOrSpeed = "/value";
    } else {
        valueOrSpeed = "/speed";
    }

    int lineno = 0;
    int idx = 0;
    std::string line;
    // get index of where coordname value occurs from header (located on line 7)
    while(std::getline(in,line)){
        ++lineno;
        if(lineno == 7){
            std::stringstream stream(line);
            std::string value;
            while(std::getline(stream,value,'\t')){
                if(verbose){
                    std::cout << "header index: " << value << std::endl;
                    std::cout << "to compare:   " << (coordName+valueOrSpeed) << std::endl;
                }
                if(value.compare(coordName+valueOrSpeed)==0){
                    if(verbose){
                        std::cout << "index in header found: " << idx << std::endl;
                    }
                    break;
                }
                idx++;
            }
        }
        if(lineno > 7){
            break;
        }
    }
    in.close();
    // now get the value at that same index we found earlier but on the next
    // line (where the state values are situationed, line 8)
    std::ifstream in2(absPath + "state.sto");
    lineno = 0;
    while(std::getline(in2,line)){
        ++lineno;
        if(lineno == 8){
            std::stringstream stream(line);
            std::string value;
            int idxCheck = 0;
            while(std::getline(stream,value,'\t')){
                if(verbose){
                    std::cout << "idx: " << idx << std::endl;
                    std::cout << "idxCheck: " << idxCheck << std::endl;
                }
                if (idx == idxCheck){
                    if(verbose){
                        std::cout << "coord from state found: " << value << std::endl;
                    }
                    in2.close();
                    return std::stod(value);
                }
                idxCheck++;
            }
        }
    }
    in2.close();
    return 0.0;
}
