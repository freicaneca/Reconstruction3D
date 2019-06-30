#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
 
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

        Point2f epipole1,epipole2;

        epipole1.x=(epilines1[0][1]*epilines1[1][2] - epilines1[0][2]*epilines1[1][1])/(epilines1[0][0]*epilines1[1][1] - epilines1[0][1]*epilines1[1][0]);

        epipole1.y=(epilines1[0][2]*epilines1[1][0] - epilines1[0][0]*epilines1[1][2])/(epilines1[0][0]*epilines1[1][1] - epilines1[0][1]*epilines1[1][0]);
        

        epipole2.x=(epilines2[0][1]*epilines2[1][2] - epilines2[0][2]*epilines2[1][1])/(epilines2[0][0]*epilines2[1][1] - epilines2[0][1]*epilines2[1][0]);

        epipole2.y=(epilines2[0][2]*epilines2[1][0] - epilines2[0][0]*epilines2[1][2])/(epilines2[0][0]*epilines2[1][1] - epilines2[0][1]*epilines2[1][0]);



        

        cv::Mat proj1 = (Mat_<double>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,0,1,0);


        cv::Mat epiCrossMat = (Mat_<double>(3,3) << 0,-1,epipole1.y,1,0,-epipole1.x,-epipole1.y,epipole1.x,0);

        cv::Mat proj2;

        cv:: Mat epiMat = (Mat_<double>(3,1) << epipole1.x,epipole1.y,epipole1.x,1);

        cv:: Vec<float,3> points3d;
        

        cv::hconcat(epiCrossMat*F, epiMat, proj2);

        

        cv::triangulatePoints(proj1,proj2,points1,points2,points3d);

        




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
 
 
