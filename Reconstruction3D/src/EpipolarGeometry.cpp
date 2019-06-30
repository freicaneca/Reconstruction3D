/*
 * EpipolarGeometry.cpp
 *
 *  Created on: 4 de jun de 2019
 *      Author: fdbo
 */

#include "EpipolarGeometry.hpp"
#include <cmath>
#include "opencv2/core.hpp"

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
	//this->nm = cv::Mat(3, 3, CV_64F, 0.0);
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

/*
cv::Mat EpipolarGeometry::GetNormalizationMatrix()
{
	return ;
}
*/

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

	// Agora, v1 e v2 têm os pontos normalizados. Precisamos atualizar this->corresp
	// com esses novos valores. Mas v1 e v2 estão em coordenadas homogêneas.
	// Precisamos des-homogeneizar.
	cv::convertPointsFromHomogeneous(v1, this->normalizedCorresp[0]);
	cv::convertPointsFromHomogeneous(v2, this->normalizedCorresp[1]);

	/*
	for (size_t i = 0; i < this->corresp[0].size(); i++)
	{
		std::cout << this->corresp[0][i] << std::endl;
		std::cout << this->corresp[1][i] << std::endl;
	}
	*/

}

void EpipolarGeometry::CalculateFundamentalMatrix()
{
	// Este método não supõe que as correspondências estão normalizadas! Execute NormalizeCorrespondences se quiser
	// normalizar (é recomendado!)
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

	//std::cout << A << std::endl;

	// A primeira etapa é a solução do sistema linear Af = 0. A solução f é a última coluna da matriz V obtida
	// a partir do SVD de A. Logo, temos que calcular SVD de A.
	cv::Mat D;
	cv::Mat U;
	cv::Mat Vt;

	//std::cout << "antes SVDecomp" << std::endl;

	cv::SVDecomp(A, D, U, Vt);

	//std::cout << D << std::endl;

	// Uma primeira aproximacao de F é a ultima coluna de V
	Vt = Vt.t();
	cv::Mat tmp = Vt.col(8).clone();
	//std::cout << "printando tmp" << std::endl;
	//std::cout << tmp << std::endl;
	this->f = tmp.reshape(0, 3);
	//std::cout << this->f << std::endl;
	//std::cout << this->f.size() << std::endl;
	//std::cout << "depois reshape" << std::endl;

	// Obtida a solucao f, precisamos forçar que ela tenha rank 2.
	// SVD novamente, agora de f.

	//std::cout << this->f.type() << " " << D.type() << U.type() << Vt.type() << std::endl;
	cv::Mat Vt2;
	cv::SVDecomp(this->f, D, U, Vt2);

	// Zera o último elemento de D, para forçar que a matriz reconstruida tenha rank 2.
	// D contém apenas os valores singulares (não é uma matriz)
	std::cout << D << std::endl;
	D.at<double>(2) = 0;
	cv::Mat diagonal = cv::Mat::diag(D);
	std::cout << diagonal << std::endl;

	//std::cout << this->f.size() << D.size() << U.size() << Vt.size() << std::endl;

	//std::cout << "antes multiplicacao final" << std::endl;
	//std::cout << cv::Mat::diag(D) << std::endl;
	this->f = U * diagonal * Vt2;
	//std::cout << "depois multiplicacao final" << std::endl;
	//this->f = tmp.reshape(3, 3);

	// Denormalizando a matriz F
	//std::cout << this->f << std::endl;
	this->f = this->nm2.t() * this->f * this->nm1;
	std::cout << "printando matrizes fundamentais!!!" << std::endl;
	std::cout << this->f << std::endl;

	// Testando matriz fundamental do opencv
	//cv::Mat f2 = cv::findFundamentalMat(this->corresp[0], this->corresp[1], CV_FM_8POINT);
	cv::Mat f2 = cv::findFundamentalMat(this->corresp[0], this->corresp[1], CV_FM_8POINT);

	std::cout << f2 << std::endl;

	/*
	std::cout << "printando valores singulares!" << std::endl;
	cv::SVDecomp(this->f, D, U, Vt2);
	std::cout << D << std::endl;
	cv::SVDecomp(f2, D, U, Vt2);
	std::cout << D << std::endl;
	*/

	std::vector<cv::Point3d> tmp1;
	std::vector<cv::Point3d> tmp2;

	cv::convertPointsToHomogeneous(this->corresp[0], tmp1);
	cv::convertPointsToHomogeneous(this->corresp[1], tmp2);

	std::cout << "depois convert to homg" << std::endl;

	// Teste de xt' F x = 0
	for (size_t i = 0; i < qty_corresp; i++)
	{
		cv::Mat mtmp1 = cv::Mat(tmp1[i]);
		//std::cout << "depois mtmp1" << std::endl;
		cv::Mat mtmp2 = cv::Mat(tmp2[i]);
		//std::cout << "depois mtmp2" << std::endl;
		//std::cout << mtmp1 << std::endl;
		//std::cout << mtmp2 << std::endl;

		std::cout << mtmp2.t() * this->f * mtmp1 << std::endl;
		std::cout << mtmp2.t() * f2 * mtmp1 << std::endl;
	}

	// Teste de epilinhas
	cv::Mat epitmp;
	cv::computeCorrespondEpilines(this->corresp[0], 1, this->f, epitmp);
	std::cout << epitmp << std::endl;
	cv::computeCorrespondEpilines(this->corresp[0], 1, f2, epitmp);
	std::cout << epitmp << std::endl;

	cv::computeCorrespondEpilines(this->corresp[1], 2, this->f, epitmp);
	std::cout << epitmp << std::endl;
	cv::computeCorrespondEpilines(this->corresp[1], 2, f2, epitmp);
	std::cout << epitmp << std::endl;
	/*
	std::cout << this->f.at<double>(0, 0) << " " << this->f.at<double>(0, 1) << this->f.at<double>(0, 2) << "\n" <<
			this->f.at<double>(1, 0) << " " << this->f.at<double>(1, 1) << " " << this->f.at<double>(1, 2) << "\n" <<
			this->f.at<double>(2, 0) << " " << this->f.at<double>(2, 1) << " " << this->f.at<double>(2, 2) << "\n" << std::endl;

	std::cout << std::endl;
	*/

	//std::cout << tmp.at<double>(0) << " " << tmp.at<double>(1) << std::endl;

}

// Teste de normalizacao
int main()
//int main_test()
{

	std::vector<cv::Point2d> points1;
	cv::Point2d p1(2, 2);
	cv::Point2d p2(10, 10);
	cv::Point2d p3(20, 20);
	cv::Point2d p4(25, 25);
	cv::Point2d p5(5, 5);
	cv::Point2d p6(30, 30);
	cv::Point2d p7(45, 45);
	cv::Point2d p8(50, 80);
	cv::Point2d p9(60, 20);
	cv::Point2d p10(80, 10);
	cv::Point2d p11(80, 50);
	cv::Point2d p12(90, 25);

	points1.push_back(p1);
	points1.push_back(p2);
	points1.push_back(p3);
	points1.push_back(p4);
	points1.push_back(p5);
	points1.push_back(p6);
	points1.push_back(p7);
	points1.push_back(p8);
	points1.push_back(p9);
	points1.push_back(p10);
	points1.push_back(p11);
	points1.push_back(p12);

	/*
	std::vector<cv::Point2d> points2;
	cv::Point2d q1(2.1, 2.1);
	cv::Point2d q2(2.1, 3.1);
	cv::Point2d q3(3.1, 2.1);
	cv::Point2d q4(3.1, 3.1);

	points2.push_back(q1);
	points2.push_back(q2);
	points2.push_back(q3);
	points2.push_back(q4);
	*/

	// Para teste, os pontos serão rotacionados e transladados por uma transformação conhecida.
	// Translaçao de -10 em x, -10 em y e rotacao de 15 graus.
	cv::Mat tr = cv::Mat(3, 3, CV_64F, 0.0);
	double deg = 5.0;
	double tx = -5;
	double ty = -5;
	tr.at<double>(0, 0) = cos(deg * PI / 180.0);
	tr.at<double>(0, 1) = -sin(deg * PI / 180.0);
	tr.at<double>(0, 2) = tx;
	tr.at<double>(1, 0) = sin(deg * PI / 180.0);
	tr.at<double>(1, 1) = cos(deg * PI / 180.0);
	tr.at<double>(1, 2) = ty;
	tr.at<double>(2, 2) = 1;

	// Os pontos transformados estarão em points2.
	std::vector<cv::Point2d> points2;
	std::vector<cv::Point3d> tmp;
	cv::transform(points1, tmp, tr);
	cv::convertPointsHomogeneous(tmp, points2);

	EpipolarGeometry eg;

	eg.AddCorrespondence(&p1, &(points2[0]));
	eg.AddCorrespondence(&p2, &(points2[1]));
	eg.AddCorrespondence(&p3, &(points2[2]));
	eg.AddCorrespondence(&p4, &(points2[3]));
	eg.AddCorrespondence(&p5, &(points2[4]));
	eg.AddCorrespondence(&p6, &(points2[5]));
	eg.AddCorrespondence(&p7, &(points2[6]));
	eg.AddCorrespondence(&p8, &(points2[7]));
	eg.AddCorrespondence(&p9, &(points2[8]));
	eg.AddCorrespondence(&p10, &(points2[9]));
	eg.AddCorrespondence(&p11, &(points2[10]));
	eg.AddCorrespondence(&p12, &(points2[11]));

	eg.NormalizeCorrespondences();
	eg.CalculateFundamentalMatrix();



	/*
	cv::Mat nm = eg.CalculateNormalizationMatrix(&points);
	//cv::Mat nm = eg.GetNormalizationMatrix();

	std::cout << nm.at<double>(0, 0) << " " << nm.at<double>(0, 1) << nm.at<double>(0, 2) << "\n" <<
			nm.at<double>(1, 0) << " " << nm.at<double>(1, 1) << " " << nm.at<double>(1, 2) << "\n" <<
			nm.at<double>(2, 0) << " " << nm.at<double>(2, 1) << " " << nm.at<double>(2, 2) << "\n" << std::endl;
	*/

	return 1;
}
