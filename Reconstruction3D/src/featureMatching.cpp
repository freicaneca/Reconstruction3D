#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/sfm.hpp>
 
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
 




class FCalculator{

    const int Max_feat = 1000;

    public:
    Mat im1;
    Mat im2;
    void extractAndMatch(){

        std::vector<KeyPoint> keypoints1, keypoints2;
        Mat descriptors1, descriptors2;
   

        Ptr<Feature2D> orb = ORB::create(Max_feat);
        orb->detectAndCompute(im1, Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(im2, Mat(), keypoints2, descriptors2);
        

        std::vector<DMatch> matches;
        BFMatcher matcher(NORM_HAMMING2, true);
        matcher.match(descriptors1, descriptors2, matches);
        

        
        sort(matches.begin(), matches.end());

        
        const int numGoodMatches = matches.size()*0.3;
        matches.erase(matches.begin()+numGoodMatches, matches.end());

        std::vector<Point2f> points1, points2;

        for( size_t i = 0; i < matches.size(); i++ )
        {
          points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
          points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
        }


        Mat F;



        F = findFundamentalMat(Mat(points1),Mat(points2));

        std::vector<cv::Vec<float,3>> epilines1, epilines2;
        std::vector<Point2f> newPoints1, newPoints2;

        



        computeCorrespondEpilines(points1, 1, F, epilines1);
        computeCorrespondEpilines(points2, 2, F, epilines2);




        

        cv::Mat proj1 = (Mat_<double>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0,0);



        cv::Mat proj2;


        cv:: Mat K = (Mat_<double>(3,3) << 1, 0, 0, 0, 0, 1, 0, 0, 1);


        cv:: Mat E = K.inv().t()*F*K.inv();


        cv::Mat R1,R2,t;

        decomposeEssentialMat(E, R1,  R2, t);

        hconcat(R1,t,proj2);


        proj1 = K*proj1;
        proj2 = K*proj2;

    



        Mat points3d;



        int N=points1.size();

        cv::Mat pnts3D(1,N,CV_64FC4);
        cv::Mat cam0pnts(1,N,CV_64FC2);
        cv::Mat cam1pnts(1,N,CV_64FC2);

        cam0pnts = Mat(points1);
        cam1pnts = Mat(points2);
 


        cv::triangulatePoints(proj1,proj2,cam0pnts,cam1pnts,pnts3D);









        cout<<pnts3D.col(1);







        




    }






};
 
 
int main()
{
   

  Mat im1, im2;

  im1 = imread("odo1.jpg");
  im2 = imread("odo2.jpg");
   

  FCalculator test;

  test.im1 = im1;
  test.im2 = im2;

  test.extractAndMatch();

  


   

   

   
}
 
 
