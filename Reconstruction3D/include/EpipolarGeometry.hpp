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

#include "opencv/cv.hpp"
#include <vector>
#include <iostream>

class EpipolarGeometry
{
public:

	EpipolarGeometry();
	~EpipolarGeometry();
	void CalculateFundamentalMatrix();
	void AddImage(const std::string filename);
	cv::Mat GetF();
	cv::Mat GetEpipole1();
	cv::Mat GetEpipole2();
	cv::Mat GetEpiLine1(const cv::Point2d* p);
	cv::Mat GetEpiLine2(const cv::Point2d* p);

	// Normalizacao: retorna homografia 3x3 que realiza normalização dos pontos de entrada. A normalização consiste na
	// translação dos pontos ao redor da origem (centroide é o vetor nulo) e a distância média dos pontos à origem é sqrt(2).
	cv::Mat GetNormalizationMatrix(const std::vector<cv::Point2d>* points);

	std::vector<cv::Mat> GetPairOfProjectionMatrices();
	void SetKeyPoints(const std::vector<cv::KeyPoint>* kp1, const std::vector<cv::KeyPoint>* kp2);
	void SetDescriptors(cv::InputArrayOfArrays d1, cv::InputArrayOfArrays d2);
	void SetFeatures2DAlgorithm(const cv::Feature2D* alg);

private:

	// Encontra o centroide dos pontos.
	cv::Point2d FindCentroid(const std::vector<cv::Point2d>* points);

	// Encontra módulo médio em relação ao centroide
	double FindMeanModuleAroundCentroid(const std::vector<cv::Point2d>* points, const cv::Point2d* centroid);

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

	// Vetor com as correspondências entre as duas imagens. É um vetor bidimensional, 2xM, onde M é a quantidade de correspondências
	// encontradas. corresp[0][j] e corresp[1][j] são pontos correspondentes.
	std::vector<std::vector<cv::Point2d>> corresp;

	// Epipolos.
	cv::Point3d ep1;
	cv::Point3d ep2;

	// Algoritmo para realizar detecção de pontos de interesse e cálculo de descritores.
	cv::Feature2D alg;

	// Vetor com as imagens adicionadas via AddImage. No momento, só são armazenadas duas imagens. Mais adiante, poderão
	// ser armazenadas mais imagens, de tal forma que todas elas contribuam com a reconstrução (bundle adjustment)
	std::vector<cv::Mat> images;

};



#endif /* INCLUDE_EPIPOLARGEOMETRY_HPP_ */
