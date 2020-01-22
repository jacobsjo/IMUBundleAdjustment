#include "IMUIntegration.h"
#include "BundleAdjuster.h"
#include "BundleAdjustmentIMU.h"

int main()
{
    BundleAdjustmentIMU baimu("../data/20191218_100925.csv", "../data/20191218_100925_keypoints.txt");

    int i = 0;
    ceres::Solver::Summary summary;
    while (baimu.runStep(&summary)){
        i++;
        std::cout << "Step " << i << " completed" << std::endl;
        std::cout << summary.BriefReport() << std::endl;
        std::cout << "========================================" << std::endl;
    }

    baimu.saveModel("../out/20191218_100925.off");

/*    IMUIntegration imuIntegration(15.0, "../data/20191218_100925.csv");

    int i = 0;

    while(imuIntegration.hasNextCameraState()){
        i++;
        std::cout << "Step " << i << std::endl;

        //get next camera state
        CameraState nextCameraState = imuIntegration.getNextCameraState();




        //print report
        //std::cout << summary.FullReport() << std::endl;


        imuIntegration.correctCameraState(nextCameraState);



        std::cout << "=========================================" << std::endl;
    }

    imuIntegration.saveModel("../out/20191218_100925.off");*/
}