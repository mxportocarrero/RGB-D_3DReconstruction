#ifndef VOLUMEINTEGRATOR_H
#define VOLUMEINTEGRATOR_H

#include "odometry.h"
#include "image.h"

/**
Esta Clase recibe una colecion de Point Clouds  asociadas a un
arreglo de Matrices de Transformaciones
Su objetivo es generar un mesh Global de la Escena. Lo que logra
en base al algoritmo TSDF(Truncated Signed Function) finalmente
genera la escena con variantes del algoritmo Marching Cubes
**/
class VolumeIntegrator
{
private:
    int noFrames;
    std::vector<PointCloud> PointClouds;
    std::vector<Eigen::Matrix4d> Transformations;
public:
    VolumeIntegrator(Odometry &source);

    void AlignClouds();

};

#endif // VOLUMEINTEGRATOR_H
