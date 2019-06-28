/*
 * FileExport.cpp
 *
 *  Created on: 27 de jun de 2019
 *      Author: lin
 */

#include "opencv/cv.hpp"
#include "FileExport.hpp"
#include "EpipolarGeometry.hpp"
#include <vector>
#include <iostream>
#include <fstream>

int export_to_file(std::vector<cv::Point3d> object, std::string filepath) {
	std::ofstream of;
	of.open(filepath, std::ios::binary | std::ios::trunc);
	if (!of.is_open()) {
		//could not open file!
		return -1;
	}

	int points_amount = object.size();
	int triangles_amount = points_amount / 3;

	int extra_triangle = points_amount%3 == 0 ? 0 : 1;
	int triangles_amount_including_extra = triangles_amount + extra_triangle;

	of << points_amount << " " << triangles_amount_including_extra << std::endl;
	for (auto point : object) {
		of << point.x << " " << point.y << " " << point.z << std::endl;
	}
	for (int i = 0; i < triangles_amount; ++i) {
		of << i + 1 << " " << i + 2 << " " << i + 3 << std::endl;
	}
	if (extra_triangle == 1) {
		of << points_amount - 2 << " ";
		of << points_amount - 1 << " ";
		of << points_amount << std::endl;
	}
	return 0;
}

