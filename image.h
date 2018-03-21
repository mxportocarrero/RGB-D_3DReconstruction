#ifndef IMAGE_H
#define IMAGE_H

#include "includes.h"
#include "camera.h"
#include "dataset.h"
#include "pointcloud.h"

/**
    Image, es el objeto que contiene i-esimo Par de Imagenes RGB-D respecto de un dataset
    Sincronizado y Registrado(la Relacion pixel RGB y Depth es 1:1)

    Genera por defecto un Point Cloud de los datos y estima colores y normales
**/
class Image
{
private:
    int width, height,pixelCount;
    int noframe; // number of Frame in Dataset
    // Mat para RGB y Depth
    cv::Mat RGB_frame;
    cv::Mat DEPTH_frame;
    //Camera Intrinsics
    Camera *Intrinsics;

    //Punteros a otros Objetos
    const char *folder_name;
    PointCloud *point_cloud;
    DataSet * dataset;

    void FillPointCloudData();

public:
    Image(DataSet * _dataset, int frame_number, int intrinsics);
    cv::Mat get_RGB_Mat();
    cv::Mat get_DEPTH_Mat();
    cv::Point3f get_CVCoordFromPixel(int u, int v);
    Eigen::Vector3d get_EigenCoordFromPixel(int u, int v);
    cv::Point3i get_CVColorFromPixel(int u, int v);
    const PointCloud &getGLPointCloud() const;

    vec3 getPointFromPointCloud(int k);
    vec3 getColorFromPointCloud(int k);
    vec3 getNormalFromPointCloud(int k);


    void PrintInfo();
};

#endif // IMAGE_H
