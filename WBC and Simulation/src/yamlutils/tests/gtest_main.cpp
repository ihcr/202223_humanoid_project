/**
 * @file gtest_main.cpp
 * @author Jun Li (junlileeds@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * @date 2021-11-12
 */

#include <string>
#include <iostream>
#include <gtest/gtest.h>

int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}