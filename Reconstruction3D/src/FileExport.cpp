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
#include <cmath>

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

const double triangle_radius = 0.025;
const double cos120 = cos(120 * M_PI/180);
const double sin120 = sin(120 * M_PI/180);
const cv::Point3d first_point = cv::Point3d(0, 1, 0);
const cv::Point3d second_point = cv::Point3d(sin120, cos120, 0);
const cv::Point3d third_point = cv::Point3d(- sin120 * sin120, cos120, sin120 * cos120);
const cv::Point3d fourth_point = cv::Point3d(- sin120 * sin120, cos120, - sin120 * cos120);

void write_point(cv::Point3d &base_point, const cv::Point3d &point_to_add,
				std::ofstream &of) {
	base_point.x += triangle_radius * point_to_add.x;
	base_point.y += triangle_radius * point_to_add.y;
	base_point.z += triangle_radius * point_to_add.z;
	of << base_point.x << " " << base_point.y << " " << base_point.z << std::endl;
	base_point.x -= triangle_radius * point_to_add.x;
	base_point.y -= triangle_radius * point_to_add.y;
	base_point.z -= triangle_radius * point_to_add.z;
}

void write_tetrahedron_points(cv::Point3d &point, std::ofstream &of) {
	write_point(point, first_point, of);
	write_point(point, second_point, of);
	write_point(point, third_point, of);
	write_point(point, fourth_point, of);
}

void write_triangle(int a, int b, int c, std::ofstream &of) {
	of << a << " " << b << " " << c << std::endl;
}

void write_tetrahedron_triangles(int i, std::ofstream &of) {
	i = i*4 + 1;
	write_triangle(i, i+1, i+2, of);
	write_triangle(i, i+2, i+3, of);
	write_triangle(i, i+1, i+3, of);
	write_triangle(i+1, i+2, i+3, of);
}

int export_to_file(std::vector<cv::Point3d> &object, std::string filepath) {
	std::ofstream of;
	of.open(filepath, std::ios::binary | std::ios::trunc);
	if (!of.is_open()) {
		//could not open file!
		return -1;
	}

	normalize_points(object);

	int points_amount = object.size() * 4;
	int triangles_amount = points_amount;

	of << points_amount << " " << triangles_amount << std::endl;
	for (auto point : object) {
		write_tetrahedron_points(point, of);
	}
	for (unsigned int i = 0; i < object.size(); ++i) {
		write_tetrahedron_triangles(i, of);
	}
	return 0;
}

int export_to_default_file(std::vector<cv::Point3d> &object) {
	return export_to_file(object, "object.byu");
}
