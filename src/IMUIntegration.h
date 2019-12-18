#pragma once

#include <Eigen/Dense>
#include <list>

struct PathState{
    PathState(Eigen::Vector3d _position, Eigen::Vector3d _orientation):
        position(_position), orientation(_orientation) {};
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};


class IMUIntegration {
public:
    IMUIntegration();
    IMUIntegration(PathState initalPathState);
    void addImuStep(Eigen::Vector3d deltaPosition, Eigen::Vector3d deltaOrientation);

    PathState getPathState(float time);
    void setPathState(float time, PathState pathState);

private:
    std::list<PathState> path;
};