#include "IMUIntegration.h"


IMUIntegration::IMUIntegration() : IMUIntegration(CameraState(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond())) {}
IMUIntegration::IMUIntegration(CameraState initalPathState) : path {initalPathState} {
    lastCorrectedFrame = initalPathState.frame;
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



