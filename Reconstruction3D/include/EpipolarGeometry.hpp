/*
 * EpipolarGeometry.hpp
 *
 *  Created on: 4 de jun de 2019
 *      Author: fdbo
 *
 *  Esta classe realiza as principais rotinas da geometria epipolar. Ela pode realizar todas as operações do zero,
 *  utilizando um dos algoritmos cv::Feature2D para detecção, cálculo de descritores e realização de matches, mas
 *  também pode receber já os pontos de interesse, descritores e/ou matches previamente calculados por qualquer
 *  algoritmo.
 *
 */

#ifndef INCLUDE_EPIPOLARGEOMETRY_HPP_
#define INCLUDE_EPIPOLARGEOMETRY_HPP_

#include <opencv/cv.hpp>

class EpipolarGeometry
{
public:

	EpipolarGeometry();
	~EpipolarGeometry();
	void CalculateFundamentalMatrix();
	cv::Mat GetF();
	cv::Mat GetEpipole1();
	cv::Mat GetEpipole2();
	cv::Mat GetEpiLine1(cv::Point2d* p);
	cv::Mat GetEpiLine2(cv::Point2d* p);
	std::vector<cv::Mat> GetPairOfProjectionMatrices();
	void SetKeyPoints(std::vector<cv::KeyPoint>* kp1, std::vector<cv::KeyPoint>* kp2);
	void SetDescriptors(cv::Mat* d1, cv::Mat* d2);
	void SetFeatures2DAlgorithm(cv::Feature2D* alg);

private:

	// Matrizes de projeção das duas câmeras. Dimensão 3x4.
	cv::Mat p1;
	cv::Mat p2;

	// Descritores dos pontos de interesse das duas imagens. São matrizes NxL, onde N é a quantidade de
	// pontos de interesse, e L é a dimensão do descritor. São usados para encontrar os matches de
	// pontos de interesse.
	cv::Mat d1;
	cv::Mat d2;

	// Matriz fundamental. Dimensão 3x3.
	cv::Mat f;

	// Vetores com os pontos de interesse das duas imagens
	std::vector<cv::KeyPoint> kp1;
	std::vector<cv::KeyPoint> kp2;

	// Epipolos.
	cv::Point3d ep1;
	cv::Point3d ep2;

	// Algoritmo para realizar detecção de pontos de interesse e cálculo de descritores.
	cv::Feature2D alg;

};



#endif /* INCLUDE_EPIPOLARGEOMETRY_HPP_ */
