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
    std::vector<glm::mat4> GLTransformations;

    /*buffers to be draw*/
    int noPoints;
    std::vector<vec3> AlignedPoints;
    std::vector<vec3> AlignedNormals;
    std::vector<vec3> AlignedColors;

public:
    VolumeIntegrator(Odometry &source);

    void AlignClouds(); // Funcion que calcula las matrices de transformacion y agrega los puntos a un vector global

    int TotalPoints();

    vec3* point_data();
    vec3* color_data();
    vec3* normal_data();

    void PrintInfo();

};

#endif // VOLUMEINTEGRATOR_H
