/**
Archivo Main!! ---> la carpeta tiene Git
La forma en como compilar deberia ser la q sigue
g++ main.cpp -o main && ./main
**/

#include "includes.h"
#include "loadshader.h"
#include "dataset.h"
#include "image.h"
#include "posegraph.h"
#include "odometry.h"
#include "volumeintegrator.h"
#include "tsdf.h"

#define DATABASE_NAME "data/burghers_sample_png"
//#define DATABASE_NAME "data/cactusgarden_png"

// Seleccionamos los frames que vamos a procesar
float speed = 0.01f;
float zFactor = 1.0f;
/** FUNCIONES PROPIAS DEL PROGRAMA **/
VolumeIntegrator *integrator;

/** VARIABLES GLOBALES PARA OPENGL **/
enum VAO_IDs{GlobalPointCloud,NumVAOs};
enum Buffer_IDs{PointBuffer,ColorBuffer,NormalBuffer,NumBuffers};

GLuint VAOs[NumVAOs];
GLuint Buffers[NumBuffers];

GLuint program; // shader program

/** Variables para la Visualizacion **/
mat4 Model,View,Projection,MVP;
mat4 ModelViewMatrix;

vec3 ViewPos = vec3(5.0f,5.0f,8.0f);
vec3 RefPoint = vec3(0.0f,0.0f,0.0f);
vec3 Up = vec3(0.0f,0.0f,1.0f);

/** Funcion para setear el MVP **/
void setMVP(){
    // Model
    //Model = glm::translate(mat4(),vec3(0.0f,0.0f,0.0f));
    float scale = 1.0f;
    Model = glm::rotate(mat4(),180.0f,vec3(1,0,0)); //* glm::scale(mat4(),vec3(scale,scale,scale));

    // View
    View = glm::lookAt(ViewPos,RefPoint,Up);


    // Projection
    // Matriz de Proyección: Campo de vision 45°, Ratio 4:3, display range: 0.1 unit<->100 units
    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    Projection = glm::perspective(glm::radians(50.0f)*zFactor, 4.0f / 4.0f, 0.1f, 150.0f);

    MVP = Projection * View * Model;
}

/** Funcion Init **/
void Init(){
    glClearColor(0.0f,0.0f,0.0f,0.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    program = LoadShaders("vertex.shader","fragment.shader");

    glGenVertexArrays(NumVAOs,VAOs);
    glBindVertexArray(VAOs[GlobalPointCloud]);

    // Generamos 3 buffers
    // Estos buffers son Estaticos los haremos dinamicos, por mientras esta bien
    glGenBuffers(NumBuffers,Buffers);
    glBindBuffer(GL_ARRAY_BUFFER,Buffers[PointBuffer]);
    glBufferData(GL_ARRAY_BUFFER, 3 * integrator->TotalPoints() * sizeof(float),integrator->point_data(),GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER,Buffers[ColorBuffer]);
    glBufferData(GL_ARRAY_BUFFER, 3 * integrator->TotalPoints() * sizeof(float),integrator->color_data(),GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER,Buffers[NormalBuffer]);
    glBufferData(GL_ARRAY_BUFFER, 3 * integrator->TotalPoints() * sizeof(float),integrator->normal_data(),GL_DYNAMIC_DRAW);
}
/** Funcion Register **/
void RegisterVariables(){
    uniformRegister(MVP,program,"MVP");
}

/** Funcion Display **/
void display(){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glPointSize(1.0f);

    setMVP();

    // Drawing Global Point Clouds
    glUseProgram(program);
    RegisterVariables();

    glBindVertexArray(VAOs[GlobalPointCloud]);

    glBindBuffer( GL_ARRAY_BUFFER, Buffers[PointBuffer] );
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(vec3),BUFFER_OFFSET(0));

    glBindBuffer( GL_ARRAY_BUFFER, Buffers[ColorBuffer] );
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(vec3),BUFFER_OFFSET(0));

    glBindBuffer( GL_ARRAY_BUFFER, Buffers[NormalBuffer] );
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(vec3),BUFFER_OFFSET(0));

    // Drawing Function
    glDrawArrays(GL_POINTS,0,integrator->TotalPoints());

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glutSwapBuffers();

    glFlush();
}
/**FUNCION PARA KEYBOARD **/
void keyboard(unsigned char key, int x, int y){
    vec3 view_vector = RefPoint - ViewPos; // Para la referencia
    vec3 ortho_vector = vec3(0,0,0); ortho_vector.x = -view_vector.y; ortho_vector.y = view_vector.x;
    switch(key) {
    case 033: // Escape Key
    case 'q': case 'Q':
        exit(EXIT_SUCCESS );
        break;
    case 'w'://Forward move
        ViewPos.x += view_vector.x * speed;
        ViewPos.y += view_vector.y * speed;
        ViewPos.z += view_vector.z * speed;

        RefPoint.x += view_vector.x * speed;
        RefPoint.y += view_vector.y * speed;
        RefPoint.z += view_vector.z * speed;

        break;
    case 's':
        ViewPos.x -= view_vector.x * speed;
        ViewPos.y -= view_vector.y * speed;
        ViewPos.z -= view_vector.z * speed;

        RefPoint.x -= view_vector.x * speed;
        RefPoint.y -= view_vector.y * speed;
        RefPoint.z -= view_vector.z * speed;
        break;
    case 'a':
        ViewPos.x += ortho_vector.x * speed;
        ViewPos.y += ortho_vector.y * speed;
        RefPoint.x += ortho_vector.x * speed;
        RefPoint.y += ortho_vector.y * speed;
        break;
    case 'd':
        ViewPos.x -= ortho_vector.x * speed;
        ViewPos.y -= ortho_vector.y * speed;
        RefPoint.x -= ortho_vector.x * speed;
        RefPoint.y -= ortho_vector.y * speed;
        break;
    }
    //cout << "My Position = (" << ViewPos.x << ","<<ViewPos.y << ","<<ViewPos.z<<")\n";
    glutPostRedisplay();
}
//----------------------------------------------------------------------------
/**FUNCION MOUSE **/
void mouseCallback(int button, int state, int x,int y){
    if ((button == 3) || (button == 4)) // It's a wheel event
   {
       // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
       if (state == GLUT_UP) return; // Disregard redundant GLUT_UP events

       if(button == 3) zFactor > 2? zFactor = 2.0: zFactor+=0.1; else zFactor < 0? zFactor = 0: zFactor-= 0.1;
       //printf("Scroll %s At %d %d\n", (button == 3) ? "Up" : "Down", x, y);

   }
   glutPostRedisplay();
}

/** MOVIMIENTO DEL MOUSE **/
void mouseMotion(int x, int y){
    float mid_x = windowDIM /2.0f,mid_y=windowDIM/2.0f;
    float angle_x = 0.0f;
    float angle_z = 0.0f;

    if( x == mid_x && y == mid_y) return;

    glutWarpPointer(mid_x,mid_y);

    // Get the direction from the mouse cursor, set a resonable maneuvering speed
    angle_x = (float)( (mid_x - x) ) / 5000;
    angle_z = (float)( (mid_y - y) ) / 500;

    //Cambiando el valor en el eje de altura
    RefPoint.z += angle_z * 2;

    // limit the rotation around the x-axis
    float botLimit = 16;
    if((RefPoint.z - ViewPos.z) > 8)  RefPoint.z = ViewPos.z + 8;
    if((RefPoint.z - ViewPos.z) < -botLimit)  RefPoint.z = ViewPos.z - botLimit;

    vec3 view_vector = RefPoint - ViewPos;

    vec3 &mView = RefPoint;
    vec3 &mPos = ViewPos;
    vec3 &vVector = view_vector;

    mView.y = (float)(mPos.y + sin(-angle_x)*vVector.x + cos(-angle_x)*vVector.y);
    mView.x = (float)(mPos.x + cos(-angle_x)*vVector.x - sin(-angle_x)*vVector.y);

    glutPostRedisplay();
}


int main(int argc, char** argv){
    // Obtener From-->To Frames
    int from = std::atoi(argv[1]);
    int to = std::atoi(argv[2]);
    cout << from << to <<endl;

    /** Lectura y Procesamiento de Datos con OpenCV **/
    std::srand( unsigned( std::time(0) ));
    /** Parte del calculo de OpenCV */
    DataSet myDataSet(DATABASE_NAME);

    //Image img(&myDataSet,0);
    //img.PrintInfo();

    std::vector<Image> voImages;
    for(int i = from; i  <= to ;i++){
        voImages.push_back(Image(&myDataSet,i));
    }

    Odometry odometry(&voImages);
    odometry.CalcTransformations();

    integrator = new VolumeIntegrator(odometry);
    integrator->AlignClouds();
    //integrator->PrintInfo();

    /**Probar este cdigo**/
    TSDF tsdf(odometry,eVector3f(0,0,0),10.0f,16);


    /** OpenGL **/

    glutInit(&argc,argv);

    glewExperimental = GL_TRUE;

    glutInitDisplayMode( GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(windowDIM,windowDIM);
    glutInitContextVersion(4,3);
    glutInitContextProfile(GLUT_CORE_PROFILE);
    glutCreateWindow("Aligned Point Cloud");
    if(glewInit()){
        std::cerr << "No se pudo inicializar GLEW" << endl;
        exit(EXIT_FAILURE);
    }

    Init();

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouseCallback);
    glutPassiveMotionFunc(mouseMotion);

    glutMainLoop();
}
