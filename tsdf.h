#ifndef TSDF_H
#define TSDF_H

#include "includes.h"
#include "volumeintegrator.h"
#include "octree.h"

/**
Esta clase mantendrá la TSDF global e irá integrando las imagenes frame-to-frame
**/


class TSDF : public VolumeIntegrator
{
private:
    OcTree<float> *volume; // Aqui iremos acumulando las TSDF


public:
    TSDF(Odometry &Source, eVector3f center, float halfDist, int maxDepth);

};

#endif // TSDF_H
