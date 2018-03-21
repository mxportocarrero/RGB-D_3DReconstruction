#include "volumeintegrator.h"

VolumeIntegrator::VolumeIntegrator(Odometry &source)
{
    int n = source.numberOfFrames();
    noFrames = n;
    //Extraemos los PointClouds para cada frame
    for(int i = 0; i < n ; i++)
        PointClouds.push_back(source.getImage(i).getGLPointCloud());
    //Verifiquemos el contenido
    /**
    for(int i = 10000 ; i < 50000;i++){
        auto p = PointClouds[0].Points[i];
        cout << "(" << p.x << "," <<
                p.y << "," <<
                p.z << ")";
    } cout << endl;
    **/


    //Extraemos las matrices de Transformacion (n-1) matrices
    //Calculamos las Matrices verdaderas
    //Recordemos que las transformaciones  hasta ahora fueron calculadas entre pares
    // y q ellas alineaban un Target Frame a un Source Frame
    // luego para hallar la i esima matriz, debemos "acumular"(multiplicar) estas matrices
    // pero en Sentido Inverso es Podemos pensarlo como una forma recursiva
    // T_0 = T_0
    // T_k = T_k-1 * T'_k
    Transformations.resize(n-1);
    Transformations[0] = source.getTransformation(0); //Matriz base
    //cout << "Eigen transfor: " << endl << Transformations[0] << endl;

    for(int i = 1; i < n-1  ; i++)
        Transformations[i] = Transformations[i-1] * source.getTransformation(i);
        //Transformations[i] = source.getTransformation(i);

    //Insertamos una matrix identidad al principio de nuestras transformaciones
    //para emparejar nuestros vectores y hacer bucles mas facil
    Transformations.insert(Transformations.begin(),Eigen::Matrix4d::Identity());

    //Como esta clase se encarga basicamente de la visualización
    //Trabajaremos todo en base a la Libreria OpenGL
    //por lo q nuestras matrices deberan usar matrices glm
    GLTransformations.resize(n);
    for(int i = 0; i < n;i++)
        for(int x = 0; x < 4; x++)
            for(int y = 0; y < 4; y++)
                GLTransformations[i][x][y] = Transformations[i](y,x);
                //OJO: la matriz4 de GLM se llena al revez!!!!!

    FOR(i,2){
        cout << "Eigen transfor: " << endl << Transformations[i] << endl;
        printGLMatrix(GLTransformations[i]);
    }
}

void VolumeIntegrator::AlignClouds()
{
    noPoints = 0;
    int noPixels = 640*480;

    for(int i = 0; i < noFrames; i++){
        //Alineamos y transformamos todos los puntos,diferentes de cero, usando
        //las transformaciones precalculadas anteriormente
        //cout << "Frame: " << i << endl;
        int count = 0;
        for(int k = 0; k < noPixels;k++){
            vec4 p = vec4(PointClouds[i].Points[k],1.0);
            if(p.z != 0.0f && p.z < 3.0f){ // Esto es lo maximo de profundidad
                vec3 color = PointClouds[i].Colors[k];
                //cout << color.r << " " << color.g << " " << color.b <<endl;
                vec3 normal = PointClouds[i].Normals[k];

                //if(count < 5)
                //    printGLVector(vec3(p));
                p = GLTransformations[i] * p;
                //if(count < 5)
                //    printGLVector(vec3(p));
                AlignedPoints.push_back(vec3(p));
                AlignedColors.push_back(color);
                AlignedNormals.push_back(normal);

                noPoints++;
                count++;
            }
        }
    }
}

int VolumeIntegrator::TotalPoints()
{
    return noPoints;
}

vec3* VolumeIntegrator::point_data()
{
    return AlignedPoints.data();
}

vec3* VolumeIntegrator::color_data()
{
    return AlignedColors.data();
}

vec3* VolumeIntegrator::normal_data()
{
    return AlignedNormals.data();
}

void VolumeIntegrator::PrintInfo()
{
    cout << "Numero de Puntos Totales: " << AlignedPoints.size() << endl;

    for(int i = 0 ; i < 1000;i++){
        cout << "(" << AlignedPoints[i].x << "," <<
                AlignedPoints[i].y << "," <<
                AlignedPoints[i].z << ")";
    } cout << endl;


}
