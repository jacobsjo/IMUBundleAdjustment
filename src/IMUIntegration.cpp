#include "IMUIntegration.h"


IMUIntegration::IMUIntegration() : IMUIntegration(PathState(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0))) {}
IMUIntegration::IMUIntegration(PathState initalPathState) : path {initalPathState} {}

void IMUIntegration::addImuStep(Eigen::Vector3d deltaPosition, Eigen::Vector3d deltaOrientation) {

}

PathState IMUIntegration::getPathState(float time) {
    return PathState(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0));
}

void IMUIntegration::setPathState(float time, PathState pathState) {

}



