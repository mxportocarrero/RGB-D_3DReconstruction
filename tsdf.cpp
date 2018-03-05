#include "tsdf.h"

TSDF::TSDF(Odometry &Source, eVector3f center, float halfDist, int maxDepth): VolumeIntegrator(Source)
{
    volume = new OcTree<float>(center,halfDist,maxDepth);
    cout << "TSDF Creado con Exito\n";
}
