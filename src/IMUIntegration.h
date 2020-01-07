#pragma once
#include <Eigen/Dense>
#include <list>
#include <stdexcept>

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include "CameraState.h"

class IMUIntegration {
public:
    /**
     * Initializes the IMUINtegration with default initial cameraState (position 0,0,0; oritentation 0;0;0)
     */
    IMUIntegration();

    /**
     * Initializes the IMUIntegration with a given initial cameraState
     * @param initalCameraState initial cmaeraState
     */
    IMUIntegration(CameraState initalCameraState);


    /**
     * Initializes the IMUIntegration with imu data from csv file given and default initial cameraState (position 0,0,0; oritentation 0;0;0)
     * @param filename cvs file to read imu data from
     */
    IMUIntegration(std::string filename);

    /**
     * Initializes the IMUIntegration with imu data from csv file given and initial cameraState
     * @param initalCameraState  initial cmaeraState
     * @param filename cvs file to read imu data from
     */
    IMUIntegration(CameraState initalCameraState, std::string filename);

    /**
     * Adds a new IMU meassurement. The meassurements have to be added in order
     * @param imuStep step to be added
     */
    void addImuStep(ImuStep imuStep);

    /**
     * gets the camera state for a given time
     * @param time the frame for witch the camera state is to be calculated
     * @return the camera state at the given time
     */
    CameraState getCameraState(int frame);


    /**
     * Sets a cameraState. The cameraStates have to be set in order and imustep for the time of the cameraState has to exist
     * @param cameraState the cameraState to be added.
     */
    void setCameraState(CameraState cameraState);

    /**
     * Saves the path as a 3d mesh
     * @param file to save the mesh to
     */
    void saveModel(std::string filename);

private:



    std::list<CameraState> path; //ordered list of camera states
    int lastCorrectedFrame; //time of last corrected cameraState
    Eigen::Vector3d lastVelocity;
    //std::list<ImuStep> uncorrectedImuSteps; //ordered list of uncorrected IMU steps, corresponding to uncorrectedPath //TOOD: is this neeeded?
};