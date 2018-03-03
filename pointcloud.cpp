#include "pointcloud.h"

PointCloud::PointCloud()
{
    //Por el momento supongamos el valor normal de imagen
    // 640 x 480

    width = FrameWidth;
    height = FrameHeight;
}

int PointCloud::NumberOfPixels()
{
    return width * height;
}
