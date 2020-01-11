#include "IMUIntegration.h"

int main()
{
    IMUIntegration imuIntegration(15.0, "../data/20191218_100925.csv");

    while(imuIntegration.hasNextCameraState()){
        CameraState nextCameraState = imuIntegration.getNextCameraState();

        //Correct nextCameraState here!

        imuIntegration.correctCameraState(nextCameraState);
    }

    imuIntegration.saveModel("../out/20191218_100925.off");
}