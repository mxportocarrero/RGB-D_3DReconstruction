//Definimos algunas funciones adicionales
#include "includes.h"

float simpleEuclidean(cv::Point &p, cv::Point &q){
    cv::Point diff = p - q;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

float simpleEuclidean(cv::Point3f &p,cv::Point3f &q){
    cv::Point3f diff = p - q;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}
