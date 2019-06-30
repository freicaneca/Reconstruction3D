/*
 * main.cpp
 *
 *  Created on: 4 de jun de 2019
 *      Author: fdbo
 *
 *  Pipeline da reconstrucao a partir de 2 imagens:
 *
 *  1- Deteccao de pontos de interesse nas duas imagens;
 *  2- Calculo de features desses pontos;
 *  3- Match entre os pontos das imagens;
 *  4- Calculo da matriz fundamental F a partir dos matches;
 *  5- Obtencao das matrizes de projecao P, P' das 2 cameras a partir de F;
 *  6- Triangulacao a partir de P, P';
 *  7- Reconstrucao afim ou metrica (extra).
 */
#include <iostream>
//#include <opencv/cv.hpp>
#include "opencv/cv.hpp"
#include <vector>

//int main()
int main_()
{

	std::cout << "Hello, fellow humans" << std::endl;
	std::cout << "Version: " << CV_VERSION << std::endl;
	std::cout << "Major version: " << CV_MAJOR_VERSION << std::endl;

	return 0;
}




