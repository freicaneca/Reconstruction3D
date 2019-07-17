#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/sfm.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

namespace calibrate{
	
	Mat calibrateCamera(int numCornersHor,int numCornersVer,string calibVideo)
	{

		int numSquares = numCornersHor * numCornersVer;
		Size board_sz = Size(numCornersHor, numCornersVer);

		VideoCapture capture = VideoCapture(calibVideo);

		vector<vector<Point3f>> object_points;
		vector<vector<Point2f>> image_points;

		vector<Point2f> corners;

		vector<Point3f> obj;
		for (int j = 0; j < numSquares; j++)
			obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

		Mat image;
		int countdown = 0;
		int numFrame = 0;


		while (capture.isOpened())
		{
			numFrame++;
			if(numFrame%10!=0)continue;

			Mat gray_image;

			capture >> image;

			if(image.empty())break;





			cvtColor(image, gray_image, CV_BGR2GRAY);
			Mat image_grayscale = gray_image.clone();
			cvtColor(image, gray_image, CV_BGR2GRAY);
			image_grayscale.convertTo(image_grayscale, CV_8U, 1 / 256.0);

 







			
			bool found = findChessboardCorners(image_grayscale, board_sz, corners);







			if (found)
			{



				image_points.push_back(corners);
				object_points.push_back(obj);

				countdown++;
				if(countdown>10)break;


			}
		}

		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat distCoeffs;
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		intrinsic.ptr<float>(0)[0] = 1;
		intrinsic.ptr<float>(1)[1] = 1;

		calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

		return intrinsic;
	}



}

namespace EpipolatGeometry
{


  Mat extractFundamentalMatrix(Mat im1, Mat im2,vector<Point2f> &points1,vector<Point2f> &points2)
  {
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    Ptr<Feature2D> orb = ORB::create(1000);
    orb->detectAndCompute(im1, Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(im2, Mat(), keypoints2, descriptors2);

    std::vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING2, true);
    matcher.match(descriptors1, descriptors2, matches);

    sort(matches.begin(), matches.end());

    const int numGoodMatches = matches.size() * 0.3;
    matches.erase(matches.begin() + numGoodMatches, matches.end());



    for (size_t i = 0; i < matches.size(); i++)
    {
      points1.push_back(keypoints1[matches[i].queryIdx].pt);
      points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    Mat F;

    F = findFundamentalMat(Mat(points1).t(), Mat(points2).t());

    return F;
  }


  void computeAndDrawEpilines(Mat im1,Mat im2,Mat F,Mat newPoints1,Mat newPoints2){


    std::vector<cv::Vec<float, 3>> epilines1, epilines2;


    computeCorrespondEpilines(newPoints1, 1, F, epilines1);
    computeCorrespondEpilines(newPoints2, 2, F, epilines2);

    std::vector<Point2f> start, end;

    for (int i = 0; i < epilines1.size(); i++)
    {

      if (epilines1[i][0] != 0 && epilines1[i][1])
      {

        line(im1, Point2f(0, -epilines1[i][2] / epilines1[i][1]), Point2f(im1.cols, -(epilines1[i][2] + epilines1[i][0] * im1.cols) / epilines1[i][1]), Scalar(0, 255, 0), 2, 8);
        line(im2, Point2f(0, -epilines2[i][2] / epilines2[i][1]), Point2f(im2.cols, -(epilines2[i][2] + epilines2[i][0] * im2.cols) / epilines2[i][1]), Scalar(0, 255, 0), 2, 8);



      }
    }


    imwrite("epiLines1.jpg", im1);
    imwrite("epiLines2.jpg", im2);

  }


  Mat get3DPoints(Mat F,Mat points1,Mat points2,Mat K){

    cv::Mat proj1 = (Mat_<double>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0);
    cv::Mat proj2;


    cv::Mat E = K.inv().t() * F * K.inv();
    cv::Mat R1, R2, t;

    decomposeEssentialMat(E, R1, R2, t);

    hconcat(R1, t, proj2);

    proj1 = K * proj1;
    proj2 = K * proj2;

    Mat points3d;



    cv::triangulatePoints(proj1, proj2, points1,points2, points3d);

    return points3d;




  }



} // namespace EpipolatGeometry



int main()
{

  Mat im1, im2;

  im1 = imread("odo1.jpg");
  im2 = imread("odo2.jpg");
  vector<Point2f> points1,points2;
  Mat F;

  F = EpipolatGeometry::extractFundamentalMatrix(im1,im2,points1,points2);

  std::vector<cv::Vec<float, 3>> epilines1, epilines2;

  Mat newPoints1, newPoints2;

  correctMatches(F, Mat(points1).t(), Mat(points2).t(), newPoints1, newPoints2);

  EpipolatGeometry::computeAndDrawEpilines(im1,im2,F,newPoints1,newPoints2);


  Mat pnts3D;

  cv::Mat K = calibrate::calibrateCamera(9,7,"video_calib.mp4");
  pnts3D = EpipolatGeometry::get3DPoints(F,newPoints1,newPoints2,K);
    
    
  for(int i=0;i<pnts3D.cols;i++){

    cout << pnts3D.col(i);
    cout<<"\n";

  }

}

