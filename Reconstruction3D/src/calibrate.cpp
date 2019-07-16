#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

class FCalculator
{

	const int Max_feat = 1000;

public:
	Mat im1;
	Mat im2;
	void extractAndMatch()
	{

		int numBoards = 0;
		int numCornersHor = 9;
		int numCornersVer = 7;

		int numSquares = numCornersHor * numCornersVer;
		Size board_sz = Size(numCornersHor, numCornersVer);

		VideoCapture capture = VideoCapture("video_calib.mp4");

		vector<vector<Point3f>> object_points;
		vector<vector<Point2f>> image_points;

		vector<Point2f> corners;

		vector<Point3f> obj;
		for (int j = 0; j < numSquares; j++)
			obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

		Mat image;
		int countdown = 0;



		while (capture.isOpened())
		{


			Mat gray_image;

			capture >> image;





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
			}
		}

		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat distCoeffs;
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		intrinsic.ptr<float>(0)[0] = 1;
		intrinsic.ptr<float>(1)[1] = 1;

		calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

		cout << intrinsic;
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
