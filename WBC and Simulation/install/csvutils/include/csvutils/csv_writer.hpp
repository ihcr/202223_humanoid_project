/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   csv_writer.hpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Header file for CSVWriter class
 *  @date   July 08, 2022
 **/

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace CSV {

class CSVWriter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::VectorXd VectorXd;
    typedef const Eigen::Ref<const VectorXd>&  ConstRefVectorXd;

    CSVWriter(const std::string& filename, const std::string& delm = ",");

    void addDataHeading(const std::vector<std::string>& heading);
    void addDataInRow(ConstRefVectorXd data);

private:
    std::string filename_;
    std::string delimeter_;
    int lines_count_;
};

}  // end CSV namespace