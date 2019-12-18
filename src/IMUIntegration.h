#ifndef IMUBUNDLEADJUSTMENT_IMUINTEGRATION_H
#define IMUBUNDLEADJUSTMENT_IMUINTEGRATION_H


class IMUIntegration {
public:
    IMUIntegration();
    void addImuStep(Vector3 deltaPosition, Vector3 deltaOrientation);

    Vector3 getPosition(float time);
    void setPosition(float time, Vector3 position);

    Vector3 getOrientation(float time);
    void setOrientation(float time, Vector3 orientation);
};


#endif //IMUBUNDLEADJUSTMENT_IMUINTEGRATION_H
