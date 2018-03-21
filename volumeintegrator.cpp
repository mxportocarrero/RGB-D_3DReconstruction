#include "volumeintegrator.h"

VolumeIntegrator::VolumeIntegrator(Odometry &source)
{
    Eigen::Vector3f center(0,0,0);
    //float dist = 65.536f;
    float dist = 32.768f;
    //float dist = 8.0f;
    int max_depth = 16;
    tsdf = new VisualOcTree<float>(center,dist,max_depth);

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
}

VolumeIntegrator::VolumeIntegrator(DataSet *_dataset, int from, int to, int intrinsics){
    Eigen::Vector3f center(0,0,0);
    //float dist = 65.536f;
    float dist = 32.768f;
    //float dist = 8.0f;
    int max_depth = 16;
    tsdf = new VisualOcTree<float>(center,dist,max_depth);



    int noPixels = 640*480;
    for(int i = from; i < to; i++){
        if(i == from){//Solo para la primera iteracion
            Transformations.push_back(Eigen::Matrix4d::Identity());
            Image s(_dataset,i,intrinsics);
            PointCloud pc = s.getGLPointCloud();

            for(int k = 0; k < noPixels;k++){
                vec4 p = vec4(pc.Points[k],1.0f);
                if(p.z != 0.0f && p.z < 3.0f){ // Esto es lo maximo de profundidad
                    vec3 color = pc.Colors[k];
                    vec3 normal = pc.Normals[k];

                    Eigen::Vector3f loc = Eigen::Vector3f(p.x,p.y,p.z);
                    Eigen::Vector3f c = Eigen::Vector3f(color.x,color.y,color.z);

                    tsdf->insert(loc,c,1.0f);
                }
            }
            GLTransformations.push_back(mat4(1.0f));

            tsdf->simplify();
            continue;
        }

        Image source(_dataset,i-1,intrinsics);
        Image target(_dataset,i,intrinsics);

        Eigen::Matrix4d T = ComputeOdometry(source,target);

        T = Transformations.back() * T;
        Transformations.push_back(T);
        mat4 GLtransform;
        for(int x = 0; x < 4; x++)
            for(int y = 0; y < 4; y++)
                GLtransform[x][y] = Transformations[i](y,x);
        GLTransformations.push_back(GLtransform);

        PointCloud pc = target.getGLPointCloud();
        for(int k = 0; k < noPixels;k++){
            vec4 p = vec4(pc.Points[k],1.0);
            if(p.z != 0.0f && p.z < 3.0f){ // Esto es lo maximo de profundidad
                vec3 color = pc.Colors[k];

                vec3 normal = pc.Normals[k];

                p = GLTransformations[i] * p;
                Eigen::Vector3f loc = Eigen::Vector3f(p.x,p.y,p.z);
                Eigen::Vector3f c = Eigen::Vector3f(color.x,color.y,color.z);
                tsdf->insert(loc,c,1.0f);
            }
        }
        tsdf->simplify();
    }
}

void VolumeIntegrator::AlignClouds()
{
    int noPixels = 640*480;

    for(int i = 0; i < noFrames; i++){
        //Alineamos y transformamos todos los puntos,diferentes de cero, usando
        //las transformaciones precalculadas anteriormente
        //cout << "Frame: " << i << endl;
        //int count = 0;
        for(int k = 0; k < noPixels;k++){
            vec4 p = vec4(PointClouds[i].Points[k],1.0);
            if(p.z != 0.0f && p.z < 3.0f){ // Esto es lo maximo de profundidad
                vec3 color = PointClouds[i].Colors[k];
                //cout << color.r << " " << color.g << " " << color.b <<endl;
                vec3 normal = PointClouds[i].Normals[k];

                //if(count < 5)
                //    printGLVector(vec3(p));
                p = GLTransformations[i] * p;
                Eigen::Vector3f loc = Eigen::Vector3f(p.x,p.y,p.z);
                Eigen::Vector3f c = Eigen::Vector3f(color.x,color.y,color.z);
                tsdf->insert(loc,c,1.0f);

                //if(count < 5)
                //    printGLVector(vec3(p));
                //AlignedPoints.push_back(vec3(p));
                //AlignedColors.push_back(color);
                //AlignedNormals.push_back(normal);

                //noPoints++;
                //count++;
            }
        }
    }
}

void VolumeIntegrator::GenerateGLGeometry(int vm, int l = 0)
{
    tsdf->generatePoints(vm,l);
}

int VolumeIntegrator::VisualMode()
{
    return tsdf->VisualMode();
}

int VolumeIntegrator::TotalPoints()
{
    return tsdf->TotalNoPoints();
}

vec3* VolumeIntegrator::point_data()
{
    return tsdf->PointData();
}

vec3* VolumeIntegrator::color_data()
{
    return tsdf->ColorData();
}

vec3* VolumeIntegrator::normal_data()
{
    return tsdf->NormalData();
}

void VolumeIntegrator::PrintInfo()
{

}
