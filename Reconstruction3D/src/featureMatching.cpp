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
        

        Mat imMatches;
        drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
        imwrite("matches.jpg", imMatches);




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
 
 
