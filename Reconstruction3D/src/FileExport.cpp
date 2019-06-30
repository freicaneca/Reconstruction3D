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

cv::Point3d centroid (std::vector<cv::Point3d>& points) {
	double cx = 0;
	double cy = 0;
	double cz = 0;

	for (auto &i : points) {
		cx += i.x;
		cy += i.y;
		cz += i.z;
	}

	int size = points.size();

	return cv::Point3d(cx/size, cy/size, cz/size);
}

double mean_modulus (std::vector<cv::Point3d>& points) {
	double mm = 0;
	for (auto &i : points) {
		auto powersum = i.x * i.x + i.y * i.y + i.z * i.z;
		mm += sqrt(powersum);
	}
	return mm / points.size();
}

void normalize_points (std::vector<cv::Point3d>& points) {
	auto cen = centroid(points);
	for (auto &i : points) {
		i.x -= cen.x;
		i.y -= cen.y;
		i.z -= cen.z;
	}
	auto mm = mean_modulus(points);
	for (auto &i : points) {
		i.x /= mm;
		i.y /= mm;
		i.z /= mm;
	}
}

int export_to_file(std::vector<cv::Point3d> object, std::string filepath) {
	std::ofstream of;
	of.open(filepath, std::ios::binary | std::ios::trunc);
	if (!of.is_open()) {
		//could not open file!
		return -1;
	}

	normalize_points(object);

	int points_amount = object.size();
	int triangles_amount = points_amount / 3;

	int extra_triangle = points_amount%3 == 0 ? 0 : 1;
	int triangles_amount_including_extra = triangles_amount + extra_triangle;

	of << points_amount << " " << triangles_amount_including_extra << std::endl;
	for (auto point : object) {
		of << point.x << " " << point.y << " " << point.z << std::endl;
	}
	for (int i = 0; i < triangles_amount; ++i) {
		of << i*3 + 1 << " " << i*3 + 2 << " " << i*3 + 3 << std::endl;
	}
	if (extra_triangle == 1) {
		of << points_amount - 2 << " ";
		of << points_amount - 1 << " ";
		of << points_amount << std::endl;
	}
	return 0;
}

