/**
Archivo Main!! ---> la carpeta tiene Git
La forma en como compilar deberia ser la q sigue
g++ main.cpp -o main && ./main
**/

#include "includes.h"
#include "dataset.h"
#include "image.h"
#include "posegraph.h"

#define DATABASE_NAME "data/burghers_sample_png"

void Init(){

}

void display(){

}


int main(){
    DataSet myDataSet(DATABASE_NAME);
    Image myImg1(&myDataSet,0);
    Image myImg2(&myDataSet,1);

    cv::Mat source = myImg1.get_RGB_Mat();
    cv::Mat target = myImg2.get_RGB_Mat();

    cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create();

    std::vector<cv::KeyPoint> keypoints_s;
    std::vector<cv::KeyPoint> keypoints_t;

    orb->detect(source,keypoints_s);
    orb->detect(target,keypoints_t);

    cv::Mat descriptor_s;
    cv::Mat descriptor_t;

    orb->compute(source,keypoints_s,descriptor_s);
    orb->compute(target,keypoints_t,descriptor_t);

    //Matching results
    cv::BFMatcher bf(cv::NORM_HAMMING,true);
    std::vector<cv::DMatch> matches;

    bf.match(descriptor_s,descriptor_t,matches);

    //Drawing the matches
    cv::Mat img_matches;
    cv::drawMatches(source,keypoints_s,target,keypoints_t,matches,img_matches);

    cv::imshow("f",img_matches);

    cv::waitKey(0);





    return 0;
}
