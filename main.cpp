/**
Archivo Main!! ---> la carpeta tiene Git
La forma en como compilar deberia ser la q sigue
g++ main.cpp -o main && ./main
**/

#include "includes.h"
#include "dataset.h"
#include "image.h"
#include "posegraph.h"
#include "odometry.h"

#define DATABASE_NAME "data/burghers_sample_png"
//#define DATABASE_NAME "data/cactusgarden_png"

void Init(){

}

void display(){

}


int main(){
    std::srand( unsigned( std::time(0) ));
    /** Parte del calculo de OpenCV */
    DataSet myDataSet(DATABASE_NAME);

    //Probar con menos de 500
    int n = 50; // Cantidad Total de Frames que vamos a analizar

    std::vector<Image> voImages;
    for(int i = 0; i  < n;i++){
        voImages.push_back(Image(&myDataSet,i));
    }

    Odometry odometry(&voImages);
    odometry.CalcTransformations();


    return 0;
}
