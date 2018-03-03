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

    //Get pixel values from matches
    std::vector<cv::Point2i>pixelMatch_s(matches.size());
    std::vector<cv::Point2i>pixelMatch_t(matches.size());
    for(int i = 0; i < matches.size();i++){
        int source_idx = matches[i].queryIdx;
        int target_idx = matches[i].trainIdx;

        pixelMatch_s[i] = (cv::Point2i)(keypoints_s[source_idx].pt);
        pixelMatch_t[i] = (cv::Point2i)(keypoints_t[target_idx].pt);
    }
    // Some Verifications prints
    //cout << pixelMatch_s[0] << " " << pixelMatch_t[0] << endl;

    // Randomly select 3 fo them, minimun for Point Adjustment SE(3)
    // In this case we random shuffle and select first 3
    std::vector<int> matchIndices(pixelMatch_s.size());
    std::iota(matchIndices.begin(),matchIndices.end(),0);

    /** INICIO DE RANSAC **/
    // Verify that pairwise distances match!
    // the 3 combinations should be similar under a certain threshold(dunno maybe <1.0)
    // also we must verify matched pixel has depth value
    float threshold = 10.0f; // Este threshold puede variar pero com tenemos imagenes a 30Hz deberia ser 0

    int iterations = 70; // Numero de iteraciones, propuesto por Fischler
    //Vectores para almacenar el Inlier count y las transformaciones respectivas
    std::vector<int> inlierCounts;
    std::vector<Eigen::Matrix4d> refinedTransformations;

    for(int it = 0; it < iterations; it++){
        cout <<endl<< "Ransac _ Iter " << it <<endl;
        std::vector<cv::Point3f> coord_sample_s(3),coord_sample_t(3);
        bool test_passed = false;
        while(!test_passed){
            std::random_shuffle(matchIndices.begin(),matchIndices.end());

            bool test1 = true;
            for(int i = 0; i < 3;i++){
                coord_sample_s[i] = source.get_CVCoordFromPixel(pixelMatch_s[ matchIndices[i] ].x,pixelMatch_s[ matchIndices[i] ].y);
                coord_sample_t[i] = target.get_CVCoordFromPixel(pixelMatch_t[ matchIndices[i] ].x,pixelMatch_t[ matchIndices[i] ].y);

                if(coord_sample_s[i].z == 0 || coord_sample_t[i].z == 0 )
                    test1 = false;
            }
            if(!test1) continue;// Empieza otra oteracion si no encuentra un buen sample


            float dist1s = simpleEuclidean(coord_sample_s[0],coord_sample_s[1]);
            float dist2s = simpleEuclidean(coord_sample_s[0],coord_sample_s[2]);
            float dist3s = simpleEuclidean(coord_sample_s[1],coord_sample_s[2]);

            float dist1t = simpleEuclidean(coord_sample_t[0],coord_sample_t[1]);
            float dist2t = simpleEuclidean(coord_sample_t[0],coord_sample_t[2]);
            float dist3t = simpleEuclidean(coord_sample_t[1],coord_sample_t[2]);

            if(abs(dist1s-dist1t) < threshold &&
                    abs(dist2s-dist2t) < threshold &&
                    abs(dist3s-dist3t) < threshold)
                    test_passed = true;
        }
        //A partir de este punto trabajamos solo con Eigen para agilizar los calculos
        //Una vez pasado el test Creamos un Obtenemos las muestras en Vector Eigen
        std::vector<Eigen::Vector3d> coord_s,coord_t;
        for(int i = 0; i < matches.size();i++){
            Eigen::Vector3d tmp_s = source.get_EigenCoordFromPixel(pixelMatch_s[ matchIndices[i] ].x,pixelMatch_s[ matchIndices[i] ].y);
            Eigen::Vector3d tmp_t = target.get_EigenCoordFromPixel(pixelMatch_t[ matchIndices[i] ].x,pixelMatch_t[ matchIndices[i] ].y);
            if(tmp_s.z() != 0.0 && tmp_t.z() != 0.0){ // Lo valores de Z en ninguno de los pares debe ser cero. //Caso contrario no se toman en cuenta
                coord_s.push_back(tmp_s);
                coord_t.push_back(tmp_t);
            }
        }
        cout << "Total Number of Posible inliers" << coord_s.size()<<endl;

        // Estimate Rotation-Translation Matrix en
        // base a los 3 primeros puntos (Los que revisamos y pasan el test)
        std::vector<Eigen::Vector3d>v_s(coord_s.begin(),coord_s.begin()+3);
        std::vector<Eigen::Vector3d>v_t(coord_t.begin(),coord_t.begin()+3);
        /** // Ayudo a verificar el funcionamento de la descomposicion SVD
        std::vector<Eigen::Vector3d>v_s;
        v_s.push_back(Eigen::Vector3d(0,0,0));
        v_s.push_back(Eigen::Vector3d(1,1,1));
        v_s.push_back(Eigen::Vector3d(2,3,4));
        std::vector<Eigen::Vector3d>v_t;
        v_t.push_back(Eigen::Vector3d(1,1,1));
        v_t.push_back(Eigen::Vector3d(2,2,2));
        v_t.push_back(Eigen::Vector3d(3,4,5));
        **/
        Eigen::Matrix4d preTransformation = QuickTransformation(v_s,v_t);

        // Count number of inliers
        int num_Inliers = 0;
        std::vector<int> InlierIndex;
        for(int i = 0; i < coord_s.size(); i++){
            Eigen::Vector3d p = coord_s[i];
            Eigen::Vector3d q = coord_t[i];
            Eigen::Vector4d v = (Eigen::Vector4d(p(0),p(1),p(2),1.0)  - preTransformation * Eigen::Vector4d(q(0),q(1),q(2),1.0));
            Eigen::Vector3d dist = Eigen::Vector3d(v(0),v(1),v(2));
            double distance = dist.norm();
            if(distance < 0.03){ // deberia ser 0.03, sabiendo que trabajamos en la escala de 1m
                num_Inliers++;
                InlierIndex.push_back(i);
            }
        }
        cout << "Numero de Inliers: " << num_Inliers << endl;
        inlierCounts.push_back(num_Inliers);

        // Refinamos la Transformacion usando solo los inliers
        std::vector<Eigen::Vector3d> final_coord_s,final_coord_t;
        for(int i = 0; i < num_Inliers; i++){
            final_coord_s.push_back(coord_s[ InlierIndex[i] ]);
            final_coord_t.push_back(coord_t[ InlierIndex[i] ]);
        }
        Eigen::Matrix4d transformation = QuickTransformation(final_coord_s,final_coord_t);
        cout << "Transformacion Refinada:" << endl << transformation << endl;
        refinedTransformations.push_back(transformation);
    } // Fin de las iteraciones RANSAC

    //Seleccionamos la mejor transformacion(aquella con mas inliers)
    int max = 0;
    int bestIteration = 0;
    for(int i = 0; i < iterations;i++){
        if(max < inlierCounts[i]){
            max = inlierCounts[i];
            bestIteration = i;
        }
    }
    cout << "==========================\n";
    cout << "La mejor Iteracion fue "<< bestIteration << endl << refinedTransformations[bestIteration]<< endl;
    cout << "==========================\n";

    //Retornamos la mejor transformacion Refinada
    return refinedTransformations[bestIteration];

}

std::vector<cv::DMatch> computeFeatureMatches(const cv::Mat &source,std::vector<cv::KeyPoint>& keypoints_s, const cv::Mat &target,std::vector<cv::KeyPoint>& keypoints_t)
{
    cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create();

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

    cv::Mat img_matches;
    cv::drawMatches(source,keypoints_s,target,keypoints_t,matches,img_matches);

    cv::imshow("features Matches",img_matches);

    cv::waitKey(10000);
    **/

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
