#include "IMUIntegration.h"

int main()
{
    IMUIntegration imuIntegration("../data/20191218_100925.csv");
    imuIntegration.saveModel("../out/20191218_100925.off");
}