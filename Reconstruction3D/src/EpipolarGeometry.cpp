/*
 * EpipolarGeometry.cpp
 *
 *  Created on: 4 de jun de 2019
 *      Author: fdbo
 */

#include "EpipolarGeometry.hpp"
#include <cmath>
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

#define PI 3.14159265

EpipolarGeometry::EpipolarGeometry()
{
	this->p1 = cv::Mat(3, 4, CV_64F, 0.0);
	this->p2 = cv::Mat(3, 4, CV_64F, 0.0);
	this->d1 = cv::Mat();
	this->d2 = cv::Mat();
	this->kp1 = std::vector<cv::KeyPoint>();
	this->kp2 = std::vector<cv::KeyPoint>();
	this->corresp = std::vector<std::vector<cv::Point2d>>(2, std::vector<cv::Point2d>());
	this->normalizedCorresp = std::vector<std::vector<cv::Point2d>>(2, std::vector<cv::Point2d>());
	this->ep1 = cv::Point3d();
	this->ep2 = cv::Point3d();
	this->images = std::vector<cv::Mat>(2);

	this->f = cv::Mat(3, 3, CV_64F, 0.0);

}

EpipolarGeometry::~EpipolarGeometry()
{

}

cv::Mat EpipolarGeometry::CalculateNormalizationMatrix(const std::vector<cv::Point2d>* points)
{
	cv::Mat nm = cv::Mat(3, 3, CV_64F, 0.0);
	cv::Point2d centroid = this->FindCentroid(points);
	double mean_module = this->FindMeanModuleAroundCentroid(points, &centroid);

	nm.at<double>(0, 0) = sqrt(2.0) / mean_module;
	nm.at<double>(1, 1) = sqrt(2.0) / mean_module;
	nm.at<double>(2, 2) = 1.0;

	nm.at<double>(0, 2) = -(sqrt(2.0) * centroid.x) / mean_module;
	nm.at<double>(1, 2) = -(sqrt(2.0) * centroid.y) / mean_module;

	return nm;

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


	return m;
}

void EpipolarGeometry::AddCorrespondence(const cv::Point2d* p1, const cv::Point2d* p2)
{
	this->corresp[0].push_back(*p1);
	this->corresp[1].push_back(*p2);
}

void EpipolarGeometry::NormalizeCorrespondences()
{
	std::vector<cv::Point3d> v1;
	std::vector<cv::Point3d> v2;
	//size_t qty_corresp = this->corresp[0].size();

	this->nm1 = this->CalculateNormalizationMatrix(&(this->corresp[0]));
	this->nm2 = this->CalculateNormalizationMatrix(&(this->corresp[1]));

	// Transformando os pontos via homografia de normalização
	cv::transform(this->corresp[0], v1, this->nm1);
	cv::transform(this->corresp[1], v2, this->nm2);

	std::cout << "depois cv transform" << std::endl;

	// Agora, v1 e v2 têm os pontos normalizados. Precisamos atualizar this->corresp
	// com esses novos valores. Mas v1 e v2 estão em coordenadas homogêneas.
	// Precisamos des-homogeneizar.
	cv::convertPointsFromHomogeneous(v1, this->normalizedCorresp[0]);
	cv::convertPointsFromHomogeneous(v2, this->normalizedCorresp[1]);


}

void EpipolarGeometry::ComputeCorrespondencesFromImages(const std::string img1, const std::string img2)
{

        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        const int maxFeat = 1000;
        cv::Ptr<cv::Feature2D> orb = cv::ORB::create(maxFeat);

        cv::Mat im1 = cv::imread(img1);
        cv::Mat im2 = cv::imread(img2);

        orb->detectAndCompute(im1, cv::Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(im2, cv::Mat(), keypoints2, descriptors2);

        std::cout << keypoints1.size() << std::endl;
        std::cout << keypoints2.size() << std::endl;

        std::vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
        matcher.match(descriptors1, descriptors2, matches);

        sort(matches.begin(), matches.end());

        const int numGoodMatches = matches.size()*0.3;
        matches.erase(matches.begin()+numGoodMatches, matches.end());

        std::vector<cv::Point2d> points1, points2;

        for( size_t i = 0; i < matches.size(); i++ )
        {
          points1.push_back( cv::Point2d(keypoints1[ matches[i].queryIdx ].pt) );
          points2.push_back( cv::Point2d(keypoints2[ matches[i].trainIdx ].pt) );
        }

        this->corresp[0] = points1;
        this->corresp[1] = points2;
}

void EpipolarGeometry::CalculateFundamentalMatrix()
{
	size_t qty_corresp = this->corresp[0].size();
	cv::Mat A(qty_corresp, 9, CV_64F, 1);

	// Preenchendo matriz A
	for (size_t i = 0; i < qty_corresp; i++)
	{
		A.at<double>(i, 0) = this->normalizedCorresp[0][i].x * this->normalizedCorresp[1][i].x;
		A.at<double>(i, 1) = this->normalizedCorresp[0][i].y * this->normalizedCorresp[1][i].x;
		A.at<double>(i, 2) = this->normalizedCorresp[1][i].x;
		A.at<double>(i, 3) = this->normalizedCorresp[0][i].x * this->normalizedCorresp[1][i].y;
		A.at<double>(i, 4) = this->normalizedCorresp[0][i].y * this->normalizedCorresp[1][i].y;
		A.at<double>(i, 5) = this->normalizedCorresp[1][i].y;
		A.at<double>(i, 6) = this->normalizedCorresp[0][i].x;
		A.at<double>(i, 7) = this->normalizedCorresp[0][i].y;

		// O elemento na ultima coluna é 1, mas a matriz ja foi inicializada com tudo 1.
	}

	// A primeira etapa é a solução do sistema linear Af = 0. A solução f é a última coluna da matriz V obtida
	// a partir do SVD de A. Logo, temos que calcular SVD de A.
	cv::Mat D;
	cv::Mat U;
	cv::Mat Vt;

	cv::SVDecomp(A, D, U, Vt);

	// Uma primeira aproximacao de F é a ultima coluna de V
	Vt = Vt.t();
	cv::Mat tmp = Vt.col(8).clone();
	this->f = tmp.reshape(0, 3);

	// Obtida a solucao f, precisamos forçar que ela tenha rank 2.
	// SVD novamente, agora de f.

	cv::Mat Vt2;
	cv::SVDecomp(this->f, D, U, Vt2);

	// Zera o último elemento de D, para forçar que a matriz reconstruida tenha rank 2.
	// D contém apenas os valores singulares (não é uma matriz)
	std::cout << D << std::endl;
	D.at<double>(2) = 0;
	cv::Mat diagonal = cv::Mat::diag(D);
	std::cout << diagonal << std::endl;

	this->f = U * diagonal * Vt2;

	// Denormalizando a matriz F
	this->f = this->nm2.t() * this->f * this->nm1;


}

// Teste de normalizacao
int main()
{




	EpipolarGeometry eg;

	eg.ComputeCorrespondencesFromImages("img1.jpg", "img2.jpg");
	eg.NormalizeCorrespondences();
	eg.CalculateFundamentalMatrix();

	return 1;
}
