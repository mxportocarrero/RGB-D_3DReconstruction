#include "odometry.h"

Odometry::Odometry(std::vector<Image> *InputVector)
{
    voImages = InputVector;
    noFrames = voImages->size();
}

void Odometry::CalcTransformations()
{
    cout << "Calculando Odometria\n";
    for(int i = 1; i < voImages->size();i++){
        cout << "***********************************\n";
        cout << "***********************************\n";
        cout << "Frame Pair: " << i <<endl;
        voTransformations.push_back( ComputeOdometry(voImages->at(i),voImages->at(i-1)) );
        cout << "***********************************\n";
        cout << "***********************************\n";
    }
}

int Odometry::numberOfFrames()
{
    return noFrames;
}

Image Odometry::getImage(int i)
{
    return voImages->at(i);
}

Eigen::Matrix4d Odometry::getTransformation(int i)
{
    return voTransformations[i];
}

Eigen::Matrix4d ComputeOdometry(Image &source, Image &target)
{
    // Compute Feature Matches between two images
    cv::Mat source_Mat = source.get_RGB_Mat();
    cv::Mat target_Mat = target.get_RGB_Mat();

    std::vector<cv::KeyPoint> keypoints_s;
    std::vector<cv::KeyPoint> keypoints_t;
    std::vector<cv::DMatch> matches = computeFeatureMatches(source_Mat,keypoints_s,target_Mat,keypoints_t);
    cout << "Numero Total de Matches: " << matches.size() << endl;

    //Get pixel values from matches
    std::vector<cv::Point2i>pixelMatch_s(matches.size());
    std::vector<cv::Point2i>pixelMatch_t(matches.size());
    for(int i = 0; i < matches.size();i++){
        int source_idx = matches[i].queryIdx;
        int target_idx = matches[i].trainIdx;

        pixelMatch_s[i] = (cv::Point2i)(keypoints_s[source_idx].pt);
        pixelMatch_t[i] = (cv::Point2i)(keypoints_t[target_idx].pt);

        // Verificamos los matches con esta funcion
        //cout << i << ": " << pixelMatch_s[i] << "," << source.get_CVCoordFromPixel(pixelMatch_s[i].x,pixelMatch_s[i].y) <<
        //        " <---> " << pixelMatch_t[i] << "," << target.get_CVCoordFromPixel(pixelMatch_t[i].x,pixelMatch_t[i].y) << endl;
    }

    //return Eigen::Matrix4d::Identity(); // Para hacer pruebas

    // Filtrado de Puntos
    // Para el ICP(Point-to-Point) debemos verificar que los matches realmente tienen buena correspondencia
    // Aprovechamos los vectores pixelMatch

    std::vector<Eigen::Vector3d> coord_s;
    std::vector<Eigen::Vector3d> coord_t;
    float threshold = 0.05f;
    FOR(i,matches.size()){
        Eigen::Vector3d s =  source.get_EigenCoordFromPixel(pixelMatch_s[i].x,pixelMatch_s[i].y);
        Eigen::Vector3d t =  target.get_EigenCoordFromPixel(pixelMatch_t[i].x,pixelMatch_t[i].y);

        //These vectors will be the total number of inliers!! not much to check only 500 each iteration(ORB Config)
        //Podemos ajustar el ultimo numero como parametro de tolerancia
        if(s.z() != 0.0f && t.z() != 0.0f && std::abs(s.norm() - t.norm())< threshold){
            coord_s.push_back(s);
            coord_t.push_back(t);
            //printEigenVector(s);printEigenVector(t);cout << "======================\n";
        }
    }

    cout << "Total number of Inliers: "  << coord_s.size() << endl;

    // ALGORITMO ICP
    // 1era Etapa
    Eigen::Matrix4d T,Tacc = Eigen::Matrix4d::Identity();
    FOR(it,5){
        T = QuickTransformation(coord_s,coord_t);
        Tacc = T * Tacc;
        cout << "Transformacion:" << endl << T << endl;
        cout << "Transformacion Acc:" << endl << Tacc << endl;
        FOR(i,5){
            printEigenVector(coord_s[i]);printEigenVector(coord_t[i]);cout << "======================\n";
        }
        double error = AvgError(coord_s,coord_t);
        cout << "Error Promedio: " << error << endl;

        // Actualizamos los valores para la siguiente iteracion
        FOR(i,coord_t.size()){
            Eigen::Vector4d tmp1 = T * Eigen::Vector4d(coord_t[i](0),coord_t[i](1),coord_t[i](2),1.0);
            Eigen::Vector3d tmp2 = Eigen::Vector3d(tmp1(0),tmp1(1),tmp1(2));
            coord_t[i] = tmp2;
        }
    }

    /**

    // Alineando de 3 en 3 puntos aleatoriamente
    std::vector<int> indices(coord_s.size());
    std::iota(indices.begin(),indices.end(),0); // Fill until coord_s.size()

    std::random_shuffle(indices.begin(),indices.end());
    **/



    return Tacc; // Para hacer pruebas









}

std::vector<cv::DMatch> computeFeatureMatches(const cv::Mat &source,std::vector<cv::KeyPoint>& keypoints_s, const cv::Mat &target,std::vector<cv::KeyPoint>& keypoints_t)
{
    cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create(1000);
    //cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create(); // Saca por defecto 500 Features de los que despues debemos filtrar

    orb->detect(source,keypoints_s);
    orb->detect(target,keypoints_t);

    cv::Mat descriptor_s;
    cv::Mat descriptor_t;

    orb->compute(source,keypoints_s,descriptor_s);
    orb->compute(target,keypoints_t,descriptor_t);

    // Feature Matching using FLANN (Kd-Trees)
    // Referirse a la documentacion de OpenCV
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(6,12,1));
    std::vector<cv::DMatch> matches;
    matcher.match(descriptor_s,descriptor_t,matches);


    /**
    //Calculamos los valores maximos y minimos
    double max_dist = -1.0;double min_dist = 1000.0;
    for(int i = 0; i < descriptor_t.rows;i++){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );


    //Drawing the good matches
    cv::Mat img_matches;

     //No podemos descartar los matches porque es posible que cuando hagamos lanzemos el error
      // de Asercion donde el indice del match no puede ser mayor que el vector que almacena los matchs
    std::vector<cv::DMatch> good_matches;
    for(int i = 0; i < descriptor_t.rows;i++){
        if(matches[i].distance <= std::max(10.0f,0.02f))
            good_matches.push_back(matches[i]);
    }
    cout << good_matches.size() << endl;
    **/
    cv::Mat img_matches;
    cv::drawMatches(source,keypoints_s,target,keypoints_t,matches,img_matches);

    cv::imshow("features Matches",img_matches);

    cv::waitKey(10000);


    return matches;
}

// p & q must have same size
Eigen::Matrix4d QuickTransformation(const std::vector<Eigen::Vector3d> &p,const std::vector<Eigen::Vector3d> &q)
{

    //Compute Center of Mass for each point clouds
    Eigen::Vector3d p_center(0.0f,0.0f,0.0f);
    Eigen::Vector3d q_center(0.0f,0.0f,0.0f);

    int n = p.size();

    for(int i = 0; i < n;i++){
        p_center += p[i];
        q_center += q[i];
    }

    p_center = p_center / (double) n;
    q_center = q_center / (double) n;

    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(3,3);
    for(int i = 0; i < n; i++){
        w += (p[i] - p_center) * (q[i] - q_center).transpose();
    }

    //Eigen::MatrixXd m = Eigen::MatrixXd::Identity(4,4);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(w,Eigen::ComputeThinU | Eigen::ComputeThinV);
    //cout << "Its singular values are: " << endl  << svd.singularValues() << endl;
    //cout << "Its left singular vector are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    //cout << "Its right singular vector are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

    // Calculamos la Rotacion
    // Ojo la matriz V debe ser traspuestas antes de ser usada
    Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
    //cout << "Rotation matrix:" << endl << R << endl;

    // Calculamos la Traslacion
    Eigen::Vector3d t = p_center - R * q_center;
    //cout << "Traslation matrix:" << endl << t << endl;

    // Componemos la matriz de Transformacion
    // [ R t ]
    // [ 0 1 ]
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    for(int i = 0; i < 3;i++)
        for(int j = 0; j < 3; j++)
            transformation(i,j) = R(i,j);
    for(int i = 0 ; i < 3; i++)
        transformation(i,3) = t(i);
    //cout << "Matriz de Transformacion(Target-->Source)" << endl << transformation << endl;

    return transformation;
}
