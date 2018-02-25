#ifndef DATASET_H
#define DATASET_H

#include "includes.h"

//Clase que se encarga de leer los archivos de Texto depth.txt y rgb.txt
class DataSet
{
private:
    std::string database_name;

    std::vector<std::string> rgb_filenames;
    std::vector<std::string> depth_filenames;
public:
    DataSet(std::string folder_name);

    std::string getRGB_filename(int i);
    std::string getDEPTH_filename(int i);

    int NoFrames();

};

#endif // DATASET_H
