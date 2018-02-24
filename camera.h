#ifndef CAMERA_H
#define CAMERA_H


enum CameraType{Cam_Defaults, Cam_Sturm2012_fr1};

//Class Camera Intrinsic Properties
class Camera
{
public:
    Camera(int type = 0);

    //Camera Parameters
    float fx = 525.0,fy = 525.0;
    float cx = 319.5, cy = 239.5;
    float depthFactor = 1000.0; // How many values are scaled to 1m
};

#endif // CAMERA_H
