//
// Created by nehil on 03/04/2021.
//
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <Eigen/QR>

constexpr unsigned num_axes = 3;
constexpr unsigned num_corners = 4;
using Matrix = Eigen::Matrix<double, num_corners, num_axes>;

using MatrixTransform = Eigen::Matrix<double, 3, 3>;

int main() {
  Matrix ROI_corners;

  float lambda[4] = {723.0f, 690.0f, 945.0f, 896.0f};

  ROI_corners(0, 0) = 336*lambda[0];
  ROI_corners(0, 1) = 550*lambda[0];
  ROI_corners(0, 2) = lambda[0];
  ROI_corners(1, 0) = 851*lambda[1];
  ROI_corners(1, 1) = 706*lambda[1];
  ROI_corners(1, 2) = lambda[1];
  ROI_corners(2, 0) = 295*lambda[2];
  ROI_corners(2, 1) = 338*lambda[2];
  ROI_corners(2, 2) = lambda[2];
  ROI_corners(3, 0) = 906*lambda[3];
  ROI_corners(3, 1) = 512*lambda[3];
  ROI_corners(3, 2) = lambda[3];



  /**
   * lambda * x = K * X
   * if we consider that x is in the pixel coordinates (x, y,6 1)
   * Our aim is to translate from pixel coordinates back to the image
   * coordinates which is represented by X (x, y, z) lambda represents the
   * distance from the camera. K represents the intrinsic parameters of our
   * camera which includes the focal length, skew, x and y principle points in
   * pixels. fs_x fs_y o_x K =   0   fs_y o_y 0    0    1 If we multiple our
   * point in pixel coordinate with lambda and the inverse of intrinsic
   * parameters we will be able to get the image coordinates.
   */

// 1827.0029296875, 0.0, 1920.650390625, 0.0, 1826.805419921875, 1102.6566162109375, 0.0, 0.0, 1.0
// 504.5502014160156, 0.0, 515.2646484375, 0.0, 0.0, 504.58697509765625, 506.1897277832031, 0.0, 0.0, 0.0, 1.0, 0.0
  MatrixTransform params;
  params << 504.5502014160156, 0.0, 515.2646484375, 0.0, 504.58697509765625, 506.1897277832031, 0.0, 0.0, 1.0;
  MatrixTransform params_inv = params.inverse();

  for(size_t row=0; row < ROI_corners.size(); row++) {
      std::cout << ROI_corners.block(row,0, 1, 3) << std::endl;
      std::cout << "the camera coords : " << params_inv * ROI_corners.block(row,0, 1, 3).transpose() << std::endl;
      std::cout << "--------" << std::endl;
  }

  return 0;
}
