/*
 * FileExport.hpp
 *
 *  Created on: 27 de jun de 2019
 *      Author: lin
 */

#ifndef INCLUDE_FILEEXPORT_HPP_
#define INCLUDE_FILEEXPORT_HPP_

#include "opencv/cv.hpp"
#include <vector>
#include <string>

int export_to_file (std::vector<cv::Point3d> object, std::string filepath);

#endif /* INCLUDE_FILEEXPORT_HPP_ */
