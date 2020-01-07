#pragma once
#include <Eigen/Dense>

struct CameraState{
    CameraState(int _frame, Eigen::Vector3d _position, Eigen::Quaterniond _orientation):
            frame(_frame), position(_position), orientation(_orientation) {};
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation; //as quaternion to avoid problems with euler angles
    int frame;
};

struct ImuStep{
    ImuStep(int _frame, Eigen::Vector3d _deltaPosition, Eigen::Vector3d _deltaOrientation):
            frame(_frame), deltaPosition(_deltaPosition), deltaOrientation(_deltaOrientation) {};
    int frame;
    Eigen::Vector3d deltaPosition;
    Eigen::Vector3d deltaOrientation; //euler angles as reported by IMU
};