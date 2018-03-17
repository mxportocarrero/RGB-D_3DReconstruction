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

void printEigenVector(Eigen::Vector3d s){ cout << "[" << s(0) << "," << s(1) << "," << s(2) << "]" << endl;}

double AvgError(const std::vector<Eigen::Vector3d> & s, const std::vector<Eigen::Vector3d> & t){
    double dist = 0.0f;
    FOR(i,s.size())
        dist += (s[i] - t[i]).norm();
    dist /= s.size();

    return dist;
}
