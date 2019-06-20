/*
 * EpipolarGeometry.cpp
 *
 *  Created on: 4 de jun de 2019
 *      Author: fdbo
 */

#include "EpipolarGeometry.hpp"
#include <cmath>

EpipolarGeometry::EpipolarGeometry()
{
	this->p1 = cv::Mat(3, 4, CV_64F, 0.0);
	this->p2 = cv::Mat(3, 4, CV_64F, 0.0);
	this->d1 = cv::Mat();
	this->d2 = cv::Mat();
	this->kp1 = std::vector<cv::KeyPoint>();
	this->kp2 = std::vector<cv::KeyPoint>();
	this->corresp = std::vector<std::vector<cv::Point2d>>(2, std::vector<cv::Point2d>());
	this->ep1 = cv::Point3d();
	this->ep2 = cv::Point3d();
	this->images = std::vector<cv::Mat>(2);

	this->f = cv::Mat(3, 3, CV_64F, 0.0);
}

EpipolarGeometry::~EpipolarGeometry()
{

}

cv::Mat EpipolarGeometry::GetNormalizationMatrix(const std::vector<cv::Point2d>* points)
{
	cv::Mat out(3, 3, CV_64F, 0.0);

	cv::Point2d centroid = this->FindCentroid(points);
	double meanModule = this->FindMeanModuleAroundCentroid(points, &centroid);

	out.at<double>(0, 0) = sqrt(2.0) / meanModule;
	out.at<double>(1, 1) = sqrt(2.0) / meanModule;
	out.at<double>(2, 2) = 1.0;

	out.at<double>(0, 2) = -(sqrt(2.0) * centroid.x) / meanModule;
	out.at<double>(1, 2) = -(sqrt(2.0) * centroid.y) / meanModule;

	return out;
}

cv::Point2d EpipolarGeometry::FindCentroid(const std::vector<cv::Point2d>* points)
{
	double cx = 0.0;
	double cy = 0.0;

	for (std::vector<cv::Point2d>::const_iterator i = points->begin(); i != points->end(); i++)
	{
		cx += (*i).x;
		cy += (*i).y;
	}

	cx /= points->size();
	cy /= points->size();

	//std::cout << "centroid " << cx << " " << cy << std::endl;

	return cv::Point2d(cx, cy);
}

double EpipolarGeometry::FindMeanModuleAroundCentroid(const std::vector<cv::Point2d>* points, const cv::Point2d* centroid)
{
	double m = 0.0;

	for (std::vector<cv::Point2d>::const_iterator i = points->begin(); i != points->end(); i++)
	{
		double dx = pow((*i).x - centroid->x, 2);
		double dy = pow((*i).y - centroid->y, 2);

		m += sqrt(dx + dy);
	}

	m /= points->size();

	//std::cout << "mean module " << m << std::endl;

	return m;
}

// Teste de normalizacao
//int main()
int main_normalizacao()
{

	std::vector<cv::Point2d> points;
	cv::Point2d p1(2, 2);
	cv::Point2d p2(2, 3);
	cv::Point2d p3(3, 2);
	cv::Point2d p4(3, 3);

	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);

	EpipolarGeometry eg;

	cv::Mat nm = eg.GetNormalizationMatrix(&points);

	std::cout << nm.at<double>(0, 0) << " " << nm.at<double>(0, 1) << nm.at<double>(0, 2) << "\n" <<
			nm.at<double>(1, 0) << " " << nm.at<double>(1, 1) << " " << nm.at<double>(1, 2) << "\n" <<
			nm.at<double>(2, 0) << " " << nm.at<double>(2, 1) << " " << nm.at<double>(2, 2) << "\n" << std::endl;


	return 1;
}
