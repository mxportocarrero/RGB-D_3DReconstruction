#ifndef OCTREE_H
#define OCTREE_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>
#include <queue>

using namespace std;

/**Algunos Typedefs**/
#define FOR8 for(int i = 0; i < 8;i++)
#define for_i(n) for(int i = 0; i < n; i++)
#define for_j(n) for(int j = 0; j < n; j++)

typedef typename Eigen::Vector3f eVector3f;

/**
la configuracion para el octree es la siguiente:
* se asume un cuadrado que parte del origen
* Una profundidad predefinida d_max
* Los datos solo se guardaran en las hojas con una profundidad d_max
* nos ayudaremos con Eigen

// TODOs
 --> Agregar funcion para eliminar y actualizar nodos y puntos
**/
enum NodeType{not_leaf,leaf};
enum Octans{ooo,ooi,oio,oii,ioo,ioi,iio,iii}; // Enum para denominar los cuadrantes
/**los cuadrantes en Nodo estan dados por ek enum Octans que aprovecha la numeracion binaria o -> eje negativo, i -> eje positivo**/

/**
Bounding Box (Struct)
**/
struct BoundingBox{
    Eigen::Vector3f min,max;
    eVector3f color; // Color de la cajita inicializada en 0
    BoundingBox(Eigen::Vector3f _min,Eigen::Vector3f _max):min(_min),max(_max){
        color = eVector3f(0,0,0);
    }
};

/**
OCTREE NODE (Struct)
**/

template<typename T>
struct OcTreeNode{

    // la correspondencia entre points y data debe ser 1:1
    // Verificar este comportamiento cuando al agregar, actualizar o eliminar datos
    std::vector<T> data;
    std::vector<eVector3f> points; // Guardaremos los puntos aqui
    std::vector<eVector3f> colors; // Guardaremos los colores aqui
    int depth; // A partir del Depth buscaremos encontrar los demas parametros como las regiones, centro del voxel,
    OcTreeNode<T> *childs[8]; // Arreglo de ocho punteros // Se cuando no son hojas tienen hasta 8 hijos
    BoundingBox* bBox; // Bounding Box
    //Cada nodo del arbol tendra un atributo color

    int TotalPoints;

    Eigen::Vector3f center;
    float halfDistance;

    OcTreeNode(){}

    OcTreeNode(Eigen::Vector3f c, float dist, int _d){
        center = c;
        halfDistance = dist;
        depth = _d;
        TotalPoints = 0;

        bBox = new BoundingBox(c - Eigen::Vector3f(dist,dist,dist),c + Eigen::Vector3f(dist,dist,dist));
        //color = eVector3f(0,0,0);

        for_i(8)
            childs[i] = nullptr;
    }

    OcTreeNode<T> * hasChild(int octant){
        return childs[octant];
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

    void printBoundingBox(){
        cout << "Bounding Box: [(" << bBox->min(0) << "," << bBox->min(1)<<"," << bBox->min(2) << "),(" <<
                                    bBox->max(0) << "," << bBox->max(1) << ","<< bBox->max(2) << ")]" << endl;
    }

    BoundingBox getBoundingBox(){
        return *bBox;
    }

    void simplifyData(){
        eVector3f loc(0,0,0);
        eVector3f c(0,0,0);

        if( TotalPoints == 0){
            for_i(points.size()){
                loc += points[i];
                c += colors[i];
            }

            TotalPoints += points.size();

            loc = loc / (float) TotalPoints;
            c = c / (float) TotalPoints;

            points.clear();
            colors.clear();

            points.push_back(loc);
            colors.push_back(c);

            return;
        }

        loc = points[0] * TotalPoints; c = colors[0] * TotalPoints;

        for(int i = 1; i < points.size();i++){
            loc += points[i];
            c += colors[i];
        }

        TotalPoints += points.size() - 1;

        loc = loc / (float) TotalPoints;
        c = c / (float) TotalPoints;

        points.clear();
        colors.clear();

        points.push_back(loc);
        colors.push_back(c);
    }



    eVector3f getColor(){


        eVector3f c(0.7,0.5,0.8);

        //Insertar funcion para calcular el promedio de todos los colores de los hijos

        return c;
    }

    void printData(){
        // Imprimimos todos los valores en el Vector Data
        printBoundingBox();
        cout << "Vector Data: ";
        if(data.size() > 0){
            for_i((int)data.size())
                cout << data[i] << ", ";
            cout << endl;
        }
        else cout << "No Data in this node(voxel)" << endl;
    }
};

/**
OCTREE (Class)
**/

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
        cout << "El nodo no existe\n";
        return false;
    }

public:
    OcTree(Eigen::Vector3f c, float dist, int d){
        noNodes = 1;
        max_depth = d;
        root = new OcTreeNode<T>(c,dist,0); // Empieza el cero por que es el root
    }

    // Insertamos Data solo a nivel de las hojas
    // Cada Hoja(Voxel) lleva un Vector que almacena esta data
    bool insert(const eVector3f & loc, T _data){
        OcTreeNode<T> * p = root;
        // Assert points should be inside of octree node
        if(!isInsideNode(p,loc)){
            cout << "No se puede agregar " << _data << " la posicion " <<
             "(" <<  loc(0) << "," << loc(1) << "," << loc(2) << ") esta fuera de rango\n";
            return false;
        }

        //recorremos los niveles hasta llegar al nivel de las hojas
        //En este nivel vamos a insertar los valores
        for (int i = 0; i < max_depth; i++){
            // Determinar en que cuandrante vamos a insertar
            eVector3f dirv = loc - p->center; // Direction Vector
            //cout << dirv(0) << "," << dirv(1)<<"," << dirv(2)<< endl;
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
            if(!(p->hasChild(octant))) {
                p-> initializeChild(octant);
                noNodes++;
            }

            //Movemos el puntero hacia adelante
            p = p->childs[octant];
        }

        p->data.push_back( _data);
        p->points.push_back(loc);
        return true;
    }

    // Solo debería insertar colores a nivel de las hojas
    bool insert(const eVector3f & loc, const eVector3f & color, T _data){
        OcTreeNode<T> * p = root;
        // Assert points should be inside of octree node
        if(!isInsideNode(p,loc)){
            //cout << "error";
            //cout << "No se puede agregar " << _data << " la posicion " <<
            // "(" <<  loc(0) << "," << loc(1) << "," << loc(2) << ") esta fuera de rango\n";
            return false;
        }

        //recorremos los niveles hasta llegar al nivel de las hojas
        //En este nivel vamos a insertar los valores
        for (int i = 0; i < max_depth; i++){
            // Determinar en que cuandrante vamos a insertar
            eVector3f dirv = loc - p->center; // Direction Vector
            //cout << dirv(0) << "," << dirv(1)<<"," << dirv(2)<< endl;
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
            if(!(p->hasChild(octant))) {
                p-> initializeChild(octant);
                noNodes++;
            }

            //Movemos el puntero hacia adelante
            p = p->childs[octant];
        }

        p->data.push_back( _data);
        p->points.push_back(loc);
        p->colors.push_back(color);
        return true;
    }

    /**Extract Bounding Boxes**/
    // Si level = 0, extrae todos los boxes del arbol, caso contrario a un nivel especifico
    std::vector<BoundingBox> ExtractBoundingBoxes(int level = 0){
        std::vector<BoundingBox> boxes;
        eVector3f defaultColor(0.7,0.8,0.9);
        // Funcion tranverse para todo el arbol

        //Puntero inicial al primer nodo
        std::queue<OcTreeNode<T>*> cola;
        cola.push(root);

        if(level == 0) // Encolamos todos los hijos existentes
            while(!cola.empty()){

                for_i(8)
                    if(cola.front()->childs[i])
                        cola.push(cola.front()->childs[i]);

                BoundingBox box = cola.front()->getBoundingBox();
                box.color = defaultColor;
                boxes.push_back( box );

                cola.pop();
            }
        else
            while(!cola.empty()){
                // Encolamos todos los hijos existentes de nivel = level
                for_i(8)
                    if(cola.front()->childs[i])
                        cola.push(cola.front()->childs[i]);

                if( (level-1) == cola.front()->depth ){
                    BoundingBox box = cola.front()->getBoundingBox(); // Genera el color para el bounding box contenido en el nodo
                    box.color = getAvgColor(cola.front()); // Setea un color para el bounding box
                    boxes.push_back(box);
                }

                cola.pop();
            }

        return boxes;
    }

    // Input: Puntero a Nodo de alguna parte del árbol
    // Output: Promedio de todos los colores que derivan del nodo inicial
    // Output: El color es devuelto en formato normalizado de [0,1] no [0 255]
    eVector3f getAvgColor(OcTreeNode<T>* node){
        eVector3f c(0,0,0);
        std::vector<eVector3f> v;

        //Puntero inicial al primer nodo
        std::queue<OcTreeNode<T>*> cola;
        cola.push(node);

        while(!cola.empty()){
            for_i(8)
                if(cola.front()->childs[i])
                    cola.push(cola.front()->childs[i]);

            // Solo agregamos los colores presentes en el vector de colores de las hojas(depth = max_depth)
            if(cola.front()->depth == max_depth){
                for_i(cola.front()->colors.size())
                    v.push_back(cola.front()->colors[i]);
            }
            cola.pop();
        }

        // Promediamos los colores puestos en v

        for_i(v.size())
            c += v[i];

        return c / (float) v.size();
    }

    void simplify(){
        //Puntero inicial al primer nodo
        std::queue<OcTreeNode<T>*> cola;
        cola.push(root);

        while(!cola.empty()){
            for_i(8)
                if(cola.front()->childs[i])
                    cola.push(cola.front()->childs[i]);

            // Solo agregamos los colores presentes en el vector de colores de las hojas(depth = max_depth)
            if(cola.front()->depth == max_depth)
                cola.front()->simplifyData();

            cola.pop();
        }
    }


// Helper functions
public:
    OcTreeNode<T> * getRoot(){
        return root;
    }

    int NumberOfNodes() {return noNodes;}

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

    // Este sera un ejemplo de como recorrer todo el arbol de forma recursiva (inorder)
    void PrintOcTree(OcTreeNode<T>* node){
        if(node == root)
            cout << "\nPrinting Octree Data\n\n";
        if(!node)
            return;
        // Tenemos que recorrer todo el arbol
        cout << "d" << node->depth ;
        if(node->depth == max_depth){
            cout << endl;
            node->printData();
            return;
        }

        for_i(8){
            if(node->childs[i]){
                cout << " c" << i << " ";
                PrintOcTree(node->childs[i]);
            }

        }
    }
};

/**CONVERTIONS FUNCTIONS**/

/**
glm::vec3 eigenVec3f_To_glmVec3f(const eVector3f & v){
    return glm::vec3(v(0),v(1),v(2));
}
eVector3f glmVec3f_To_eigenVec3f(const glm::vec3 & v){
    return eVector3f(v.x,v.y,v.z);
}
**/



#endif // OCTREE_H
