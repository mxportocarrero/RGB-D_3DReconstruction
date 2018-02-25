#include "volumeintegrator.h"

VolumeIntegrator::VolumeIntegrator(Odometry &source)
{
    int n = source.numberOfFrames();
    noFrames = n;
    //Extraemos los PointClouds para cada frame
    for(int i = 0; i < n ; i++)
        PointClouds.push_back(source.getImage(i));
    //Extraemos las matrices de Transformacion (n-1) matrices
    Transformations.resize(n-1);
    Transformations[0] = source.getTransformation(0); //Matriz base
    for(int i = 1; i < n-1  ; i++)
        Transformations.push_back(source.getTransformation(i));

    //Calculamos las Matrices verdaderas
    //Recordemos que las transformaciones  hasta ahora fueron calculadas entre pares
    // y q ellas alineaban un Target Frame a un Source Frame
    // luego para hallar la i esima matriz, debemos "acumular"(multiplicar) estas matrices
    // pero en Sentido Inverso es Podemos pensarlo como una forma recursiva
    // T_0 = T_0
    // T_k = T_k-1 * T_k
}

void VolumeIntegrator::AlignClouds()
{

}
