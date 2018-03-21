#ifndef VISUALOCTREE_H
#define VISUALOCTREE_H

#include <random>

#include "includes.h"
#include "octree.h"

#include "loadshader.h"

/**
// Generar el numero flotante
float RandomFloat(float min, float max){
    // this  function assumes max > min, you may want
    // more robust error checking for a non-debug build
    assert(max > min);
    float random = ((float) rand()) / (float) RAND_MAX;

    // generate (in your case) a float between 0 and (4.5-.78)
    // then add .78, giving you a float between .78 and 4.5
    float range = max - min;
    return (random*range) + min;
}
**/

enum VisualMode{lines,cubes};

// Implementacion para Visualizar octree usando OpenGL
// Debe generar la geometría adecuada para ser visualizada en 3D
template<typename T>
class VisualOcTree : public OcTree<T>{
    //OcTree<T> * octree;

    //Buffers para la renderizacion con shaders
    int noPoints;
    int visual_mode;
    std::vector<vec3> voGLPoints;
    std::vector<vec3> voGLColors;
    std::vector<vec3> voGLNormals;
// Constructor y Funciones de OcTree
public:
    VisualOcTree(Eigen::Vector3f c, float dist, int d) :
        OcTree<T>(c,dist,d){
    }

    vec3* PointData() { return voGLPoints.data(); }
    vec3* ColorData() { return voGLColors.data(); }
    vec3* NormalData(){ return voGLNormals.data(); }

    int TotalNoPoints(){ return noPoints; }

public:
    void generatePoints(int _visual_mode,int level = 0){
        // Clear Buffer Vectors
        voGLPoints.clear();
        noPoints = 0;
        visual_mode = _visual_mode;

        //Extraer todos los bounding box
        std::vector<BoundingBox> boxes = this->ExtractBoundingBoxes(level);

        //Generar la cantidad de puntos adecuados de acuerdo a lo que se quiere visualizar
        switch(visual_mode){
        case lines:
            for_i(boxes.size()){
                LinesFromBoundingBox(boxes[i]);
                eVector3f t = boxes[i].color;
                vec3 c = vec3(t(0),t(1),t(2));
                //cout << i << "Box Color: " << c.x << " " << c.y<< " " << c.z<< endl;
            }

            break;
        case cubes:
            for_i(boxes.size()){
                CubesFromBoundingBox(boxes[i]);
                eVector3f t = boxes[i].color;
                vec3 c = vec3(t(0),t(1),t(2));
                //cout << i << " Box Color: " << c.x << " " << c.y<< " " << c.z<< endl;
            }

            break;
        }
    }

    int VisualMode(){return visual_mode;}

private:
    //Graficamos las lineas en base a GL_LINES
    void LinesFromBoundingBox(const BoundingBox & box){
        //Estraemos las esquinas a partir del boundingbox(8 Esquinas)
        //Aprovechamos el enum de Octree
        std::vector<glm::vec3> p(8);

        float xn = box.min(0) , yn = box.min(1), zn = box.min(2);
        float xp = box.max(0) , yp = box.max(1), zp = box.max(2);

        p[ooo] = vec3(xn,yn,zn);
        p[ooi] = vec3(xn,yn,zp);
        p[oio] = vec3(xn,yp,zn);
        p[oii] = vec3(xn,yp,zp);
        p[ioo] = vec3(xp,yn,zn);
        p[ioi] = vec3(xp,yn,zp);
        p[iio] = vec3(xp,yp,zn);
        p[iii] = vec3(xp,yp,zp);

        //Add Lines by pairs (12 lines edges)
        voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[oio]);
        voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[ooi]);
        voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[ooi]);
        voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[oio]);

        voGLPoints.push_back(p[ioo]);voGLPoints.push_back(p[iio]);
        voGLPoints.push_back(p[ioo]);voGLPoints.push_back(p[ioi]);
        voGLPoints.push_back(p[iii]);voGLPoints.push_back(p[ioi]);
        voGLPoints.push_back(p[iii]);voGLPoints.push_back(p[iio]);

        voGLPoints.push_back(p[ooi]);voGLPoints.push_back(p[ioi]);
        voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[iii]);
        voGLPoints.push_back(p[oio]);voGLPoints.push_back(p[iio]);
        voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[ioo]);

        noPoints += 12*2;

        eVector3f t = box.color;

        vec3 c = vec3(t(0),t(1),t(2));
        //cout << "color: " << c.x << " " << c.y<< " " << c.z<< endl;

        for_i(12*2)
            voGLColors.push_back(c);
    }

    //Graficamos el Cubo usando Quads
    void CubesFromBoundingBox(BoundingBox box){
        //Estraemos las esquinas a partir del boundingbox(8 Esquinas)
        //Aprovechamos el enum de Octree
        std::vector<glm::vec3> p(8);

        float xn = box.min(0) , yn = box.min(1), zn = box.min(2);
        float xp = box.max(0) , yp = box.max(1), zp = box.max(2);

        p[ooo] = vec3(xn,yn,zn);
        p[ooi] = vec3(xn,yn,zp);
        p[oio] = vec3(xn,yp,zn);
        p[oii] = vec3(xn,yp,zp);
        p[ioo] = vec3(xp,yn,zn);
        p[ioi] = vec3(xp,yn,zp);
        p[iio] = vec3(xp,yp,zn);
        p[iii] = vec3(xp,yp,zp);

        //Add Quads for cube drawing 6 caras faces
        voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[oio]);voGLPoints.push_back(p[ooi]);
        voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[ooi]);voGLPoints.push_back(p[oio]);

        voGLPoints.push_back(p[oio]);voGLPoints.push_back(p[iio]);voGLPoints.push_back(p[oii]);
        voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[iii]);voGLPoints.push_back(p[iio]);

        voGLPoints.push_back(p[iio]);voGLPoints.push_back(p[ioo]);voGLPoints.push_back(p[ioi]);
        voGLPoints.push_back(p[ioi]);voGLPoints.push_back(p[iii]);voGLPoints.push_back(p[iio]);

        voGLPoints.push_back(p[ioo]);voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[ooi]);
        voGLPoints.push_back(p[ooi]);voGLPoints.push_back(p[ioi]);voGLPoints.push_back(p[ioo]);

        voGLPoints.push_back(p[ooo]);voGLPoints.push_back(p[oio]);voGLPoints.push_back(p[iio]);
        voGLPoints.push_back(p[iio]);voGLPoints.push_back(p[ioo]);voGLPoints.push_back(p[ooo]);

        voGLPoints.push_back(p[ooi]);voGLPoints.push_back(p[oii]);voGLPoints.push_back(p[ioi]);
        voGLPoints.push_back(p[ioi]);voGLPoints.push_back(p[iii]);voGLPoints.push_back(p[oii]);

        noPoints += 12*3;

        //eVector3f t = eVector3f(1,0,0);
        eVector3f t = box.color;

        vec3 c = vec3(t(0),t(1),t(2));
        //cout << "color: " << c.x << " " << c.y<< " " << c.z<< endl;

        for_i(12*3)
            voGLColors.push_back(c);
    }

};



#endif // VISUALOCTREE_H
