#ifndef INCLUDES_H
#define INCLUDES_H

/**Core LIB**/

#include <iostream>
#include <numeric> // std::iota needs this lib
#include <vector>
#include <string>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <algorithm>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::fstream;
using std::cerr;

/**OPENGL LIB**/
#include <GL/glew.h>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Los dos defines de abajo me permiten imprimir directamente las matrices de glm
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/string_cast.hpp>
#include <glm/ext.hpp>

typedef glm::vec2 vec2;
typedef glm::vec3 vec3;
typedef glm::mat4 mat4;
typedef glm::mat3 mat3;
typedef glm::vec4 vec4;


#define BUFFER_OFFSET(a) ((void*)(a))
#define windowDIM 900
#define PI 3.14159265

/**OPENCV LIB**/

//Librerias opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

//Los parametros predefinidos para los datos
#define FrameWidth 640
#define FrameHeight 480

/**EIGEN LIB**/

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/LU>

/**MACROS**/

#define ENDL std::cout << std::endl;
#define FOR(i,n) for(int i = 0; i < n; i++)

/**Algunos Typedefs**/
#define FOR8 for(int i = 0; i < 8;i++)
#define for_i(n) for(int i = 0; i < n; i++)
#define for_j(n) for(int j = 0; j < n; j++)

/**Global Helper Functions**/

float simpleEuclidean(cv::Point &p,cv::Point &q);
float simpleEuclidean(cv::Point3f &p,cv::Point3f &q);

void printEigenVector(Eigen::Vector3d s);
void printGLVector(vec3 s);
void printGLMatrix(mat4 m);

double AvgError(const std::vector<Eigen::Vector3d> &s, const std::vector<Eigen::Vector3d> &t);

#endif // INCLUDES_H
