#include "IMUIntegration.h"


IMUIntegration::IMUIntegration() : IMUIntegration(CameraState(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond())) {}
IMUIntegration::IMUIntegration(CameraState initalPathState) : path {initalPathState} {
    lastCorrectedFrame = initalPathState.frame;
}

IMUIntegration::IMUIntegration(std::string filename) : IMUIntegration(CameraState(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond()), filename) {}
IMUIntegration::IMUIntegration(CameraState initalCameraState, std::string filename) : IMUIntegration(initalCameraState){
    std::ifstream infile(filename);
    std::string line = "";

    std::string row[12];

    int i = initalCameraState.frame;

    while (getline(infile, line)){

        std::stringstream strstr(line);
        std::string word = "";
        int j = 0;
        while (getline(strstr,word, ';')){
            row[j++] = word;
        }

        Eigen::Vector3d deltaPosition(stod(row[5]),stod(row[6]),stod(row[7]));
        Eigen::Vector3d deltaOrientation(stod(row[9]),stod(row[10]),stod(row[11]));

        addImuStep(ImuStep(++i,deltaPosition,deltaOrientation));

    }
}



void IMUIntegration::addImuStep(ImuStep imuStep) {
    if (path.back().frame > imuStep.frame){
        throw std::invalid_argument( "new imuStep is older than imuStep alreaddy added" );
        return;
    }

    Eigen::Vector3d newPosition = path.back().position + (path.back().orientation * imuStep.deltaPosition);

    Eigen::AngleAxisd rollAngle(imuStep.deltaOrientation[0],Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(imuStep.deltaOrientation[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(imuStep.deltaOrientation[2],Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond newOrientation = rollAngle * pitchAngle * yawAngle * path.back().orientation;

    path.push_back(CameraState(imuStep.frame, newPosition, newOrientation) );
}

CameraState IMUIntegration::getCameraState(int frame) {
    for (auto &state : path){
        if (state.frame == frame){
            return state;
        }
    }

    return CameraState(frame, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond());
}

void IMUIntegration::setCameraState(CameraState pathState) {
    if (lastCorrectedFrame > pathState.frame){
        throw std::invalid_argument( "new pathState is older than pathState alreaddy set" );
        return;
    }

    CameraState *uncorrectedPathState;

    for (auto &state : path){
        if (state.frame == pathState.frame){
            uncorrectedPathState = &state;
            break;
        }
    }

    Eigen::Vector3d positionError = uncorrectedPathState->position - pathState.position;
    Eigen::Quaterniond orientationError = uncorrectedPathState->orientation * pathState.orientation.conjugate();
    int framesToCorrect = pathState.frame - lastCorrectedFrame;

    for (auto &state : path){
        if (state.frame > lastCorrectedFrame && state.frame <= pathState.frame) {
            state.position += positionError * (state.frame - lastCorrectedFrame) / (pathState.frame - lastCorrectedFrame);
            state.orientation = state.orientation.slerp((state.frame - lastCorrectedFrame) / (pathState.frame - lastCorrectedFrame), orientationError * state.orientation);
        } else if (state.frame > pathState.frame) {
            state.orientation = orientationError * state.orientation;
            state.position = orientationError * (state.position - uncorrectedPathState->position) + pathState.position;
        }
    }

    lastCorrectedFrame = pathState.frame;
}



