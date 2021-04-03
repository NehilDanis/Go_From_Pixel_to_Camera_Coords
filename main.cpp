//
// Created by nehil on 03/04/2021.
//
#include <Eigen/Dense>
#include <array>
#include <iostream>

constexpr unsigned num_axes = 3;
constexpr unsigned num_corners = 4;
using Matrix = Eigen::Matrix<double, num_corners, num_axes>;
using IntrinsicMatrix = Eigen::Matrix<double, 3, 3>;

int main() {
  Matrix ROI_corners;
  ROI_corners(0, 0) = 14;
  ROI_corners(0, 1) = 334;
  ROI_corners(0, 2) = 1;
  ROI_corners(1, 0) = 48;
  ROI_corners(1, 1) = 154;
  ROI_corners(1, 2) = 1;
  ROI_corners(2, 0) = 586;
  ROI_corners(2, 1) = 351;
  ROI_corners(2, 2) = 1;
  ROI_corners(3, 0) = 588;
  ROI_corners(3, 1) = 180;
  ROI_corners(3, 2) = 1;

  float lambda_1 = 748.0f;
  float lambda_2 = 946.0f;
  float lambda_3 = 828.0f;
  float lambda_4 = 1020.0f;
  float lambda[4] = {748.0f, 946.0f, 828.0f, 1020.0f};

  /**
   * lambda * x = K * X
   * if we consider that x is in the pixel coordinates (x, y, 1)
   * Our aim is to translate from pixel coordinates back to the image
   * coordinates which is represented by X (x, y, z) lambda represents the
   * distance from the camera. K represents the intrinsic parameters of our
   * camera which includes the focal length, skew, x and y principle points in
   * pixels. fs_x fs_y o_x K =   0   fs_y o_y 0    0    1 If we multiple our
   * point in pixel coordinate with lambda and the inverse of intrinsic
   * parameters we will be able to get the image coordinates.
   */

// 1827.0029296875, 0.0, 1920.650390625, 0.0, 1826.805419921875, 1102.6566162109375, 0.0, 0.0, 1.0

  IntrinsicMatrix params;
  params(0, 0) = 1827.0029296875;
  params(0, 1) = 0.0;
  params(0, 2) = 1920.650390625;

  params(1, 0) = 0.0;
  params(1, 1) = 1826.805419921875;
  params(1, 2) = 1102.6566162109375;
  params(2, 0) = 0.0;
  params(2, 1) = 0.0;
  params(2, 0) = 1.0;
  IntrinsicMatrix params_inv = params.inverse();

  for(size_t row=0; row < ROI_corners.size(); row++) {
      std::cout << ROI_corners.block(row,0, 1, 3) << std::endl;
      std::cout << "the camera coords : " << params_inv * lambda[row] * ROI_corners.block(row,0, 1, 3).transpose() << std::endl;
      std::cout << "--------" << std::endl;
  }

  return 0;
}
