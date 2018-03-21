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
void printGLVector(vec3 s){ cout << "[" << s.x << "," << s.y << "," << s.z << "]" << endl;}

void printGLMatrix(mat4 m){
    cout << "GL Transfor:\n" << m[0][0] << " " << m[0][1] << " " << m[0][2] << " " << m[0][3] << endl;
    cout << m[1][0] << " " << m[1][1] << " " << m[1][2] << " " << m[1][3] << endl;
    cout << m[2][0] << " " << m[2][1] << " " << m[2][2] << " " << m[2][3] << endl;
    cout << m[3][0] << " " << m[3][1] << " " << m[3][2] << " " << m[3][3] << endl;
}

double AvgError(const std::vector<Eigen::Vector3d> & s, const std::vector<Eigen::Vector3d> & t){
    double dist = 0.0f;
    FOR(i,s.size()){

        double d = (s[i] - t[i]).norm();

        //cout << i << " source_point: "; printEigenVector(s[i]); cout<< "target_point: "; printEigenVector(t[i]); cout << "diff: " << d <<endl;
        dist += d;
    }

    dist /= s.size();

    return dist;
}
