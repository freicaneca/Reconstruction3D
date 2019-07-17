#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/highgui.hpp>

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




int main()
{

	int numCornersHor = 9;
	int numCornersVer = 7;


	Mat K = calibrate::calibrateCamera(numCornersHor,numCornersVer,"video_calib.mp4");

	cout<<K;

	
}
