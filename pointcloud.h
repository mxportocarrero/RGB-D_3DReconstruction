#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "includes.h"

// Solo es un contenedor para las Nubes de Puntos
class PointCloud
{
private:
    int width, height;
public:
    PointCloud();

    // Funciones a realizarse dentro del constructor, donde se estiman todos los datos
    void EstimatePoints();
    void EstimateNormals();
    void EstimateColors();

    vector<vec3> Points;
    vector<vec3> Normals;
    vector<vec3> Colors;
};


#endif // POINTCLOUD_H
