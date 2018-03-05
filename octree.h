#ifndef OCTREE_H
#define OCTREE_H

#include "includes.h"

typedef typename Eigen::Vector3f eVector3f;

/**
la configuracion para el octree es la siguiente:
se asume un cuadrado que parte del origen un set de niveles predefinidos
nos ayudaremos con Eigen
**/
enum NodeType{not_leaf,leaf};
enum Octans{ooo,ooi,oio,oii,ioo,ioi,iio,iii}; // Enum para denominar los cuadrantes
/**los cuadrantes en Nodo estan dados por ek enum Octans que aprovecha la numeracion binaria o -> eje negativo, i -> eje positivo**/

struct BoundingBox{
    Eigen::Vector3f min,max;
    BoundingBox(Eigen::Vector3f _min,Eigen::Vector3f _max):min(_min),max(_max){}
};

template<typename T>
struct OcTreeNode{
    std::vector<T> data;
    int depth; // A partir del Depth buscaremos encontrar los demas parametros como las regiones, centro del voxel,
    OcTreeNode<T> *childs[8]; // Arreglo de ocho punteros // Se cuando no son hojas tienen hasta 8 hijos
    BoundingBox* bBox; // Bounding Box

    Eigen::Vector3f center;
    float halfDistance;

    OcTreeNode(){}

    OcTreeNode(Eigen::Vector3f c, float dist, int _d){
        center = c;
        halfDistance = dist;
        depth = _d;

        bBox = new BoundingBox(c - Eigen::Vector3f(dist,dist,dist),c + Eigen::Vector3f(dist,dist,dist));
    }

    OcTreeNode<T> * hasChild(int octant){
        return childs[octant]; // Si no tiene el primer hijo, en realidad no tiene ninguno
    }

    void initializeChild(int octant){
        childs[octant] = new OcTreeNode<T>(center,halfDistance,depth+1);

        //Actualizamos los verdaderos valores para cada hijo
        float hd = halfDistance / 2.0f; // half distance de la halfdistance

        //Actualizamos el centro al octante correspondiente
        switch (octant){
            case ooo:
                childs[ooo]->center += eVector3f(-hd,-hd,-hd);
                break;
            case ooi:
                childs[ooi]->center += eVector3f(-hd,-hd, hd);
                break;
            case oio:
                childs[oio]->center += eVector3f(-hd,hd,-hd);
                break;
            case oii:
                childs[oii]->center += eVector3f(-hd,hd,hd);
                break;
            case ioo:
                childs[ioo]->center += eVector3f(hd,-hd,-hd);
                break;
            case ioi:
                childs[ioi]->center += eVector3f(hd,-hd, hd);
                break;
            case iio:
                childs[iio]->center += eVector3f(hd,hd,-hd);
                break;
            case iii:
                childs[iii]->center += eVector3f(hd,hd,hd);
                break;
        }
        //Actualiza la halfdistance
        childs[octant]->updateBoundingBox(hd);
    }

    void updateBoundingBox(float d){
        halfDistance = d;
        bBox->min = center - eVector3f(d,d,d);
        bBox->max = center + eVector3f(d,d,d);
    }

    void printData(){
    // Imprimimos todos los vectores correspondientes al nodo
    for_i(8)
        if(childs[i]){
            cout << "v" << i << ": ";
            for_j(childs[i]->data.size()) cout << childs[i]->data[j] << ",";
            cout << endl;
        }

    }

};

template <typename T>
class OcTree{
private:
    int noNodes;
    int max_depth; // Maximun Depth level in the Octree
    OcTreeNode<T>* root;

private:
    bool isInsideNode(OcTreeNode<T> * node, eVector3f loc){
        if(node){
            if( loc(0) < node->bBox->min(0) || loc(1) < node->bBox->min(1) || loc(2) < node->bBox->min(2) ||
                loc(0) > node->bBox->max(0) || loc(1) > node->bBox->max(1) || loc(2) > node->bBox->max(2))
            return false;
            else return true;
        }
        else
        cout << "El punto no existe\n";
        return false;
    }

public:
    OcTree(Eigen::Vector3f c, float dist, int d){
        noNodes = 1;
        max_depth = d;
        root = new OcTreeNode<T>(c,dist,0); // Empieza el cero por que es el root
    }

    // Insertamos iterativamente
    bool insert(eVector3f loc, T _data){
        OcTreeNode<T> * p = root;
        // Assert no points should be inside of the octree
        if(!isInsideNode(p,loc)) return false;

        //recorremos los niveles hasta llegar al nivel de las hojas
        for (int i = 0; i < max_depth; i++){
            // Determinar en que cuandrante vamos a insertar
            eVector3f dirv = loc - p->center; // Direction Vector
            // Obtenemos los booleanos
            bool oct[3];
            for_i(3) oct[i] = dirv(i) >= 0;

            int octant;
            if(!oct[0] && !oct[1] && !oct[2]) octant = ooo;
            if(!oct[0] && !oct[1] && oct[2]) octant = ooi;
            if(!oct[0] && oct[1] && !oct[2]) octant = oio;
            if(!oct[0] && oct[1] && oct[2]) octant = oii;
            if(oct[0] && !oct[1] && !oct[2]) octant = ioo;
            if(oct[0] && !oct[1] && oct[2]) octant = ioi;
            if(oct[0] && oct[1] && !oct[2]) octant = iio;
            if(oct[0] && oct[1] && oct[2]) octant = iii;

            // Inicializar el Child Correcto
            if(!(p->hasChild(octant))) p-> initializeChild(octant);

            //Movemos el puntero hacia adelante
            p = p->childs[octant];
        }

        p->data.push_back( _data);

    }

// Helper functions
public:
    OcTreeNode<T> * getRoot(){
        return root;
    }

    void PrintNodeInfo(OcTreeNode<T> * node){
        node->printData();
    }

    void PrintRegion(OcTreeNode<T> * node){
        // check if octant is initialized
        if(node){
            BoundingBox *b = node->bBox;
            cout << "octant bounding box\n min:(" << b->min(0) << "," << b->min(1)<< "," << b->min(2) << ")\n" <<
                    " max:(" << b->max(0) << "," << b->max(1)<< "," << b->max(2) << ")\n";

            cout << "center: (" << node->center(0) << "," << node->center(1) << "," << node->center(2) << ")\n";
        }
    }

    void PrintRegion(OcTreeNode<T> * node, int octant){
        // check if octant is initialized
        if(node->childs[octant]){
            BoundingBox *b = node->childs[octant]->bBox;
            cout << "octant child " << octant <<" bounding box\n min:(" << b->min(0) << "," << b->min(1)<< "," << b->min(2) << ")\n" <<
                    " max:(" << b->max(0) << "," << b->max(1)<< "," << b->max(2) << ")\n";
            cout << "center: (" << node->childs[octant]->center(0) << "," << node->childs[octant]->center(1) << "," << node->childs[octant]->center(2) << ")\n";
        }
    }

    void PrintLeaves(){

    }

};


#endif // OCTREE_H
