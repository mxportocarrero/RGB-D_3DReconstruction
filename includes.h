#ifndef INCLUDES_H
#define INCLUDES_H

/**Core LIB**/

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <math.h>

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
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//Los parametros predefinidos para los datos
#define FrameWidth 640
#define FrameHeight 480

/**EIGEN LIB**/

#include <eigen3/Eigen/Core>

/**MACROS**/

#define ENDL std::cout << std::endl;


#endif // INCLUDES_H
