#include "camera.h"

Camera::Camera(int type)
{
    if(type){
        switch (type) {
        case Cam_Sturm2012_fr1:
            //Camera Parameters
            fx = 517.3; fy = 516.5;
            cx = 318.6; cy = 255.3;
            depthFactor = 5000.0; // How many values are scaled to 1m
            break;
        default:
            break;
        }
    }
}
