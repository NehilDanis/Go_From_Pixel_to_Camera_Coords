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
  ROI_corners(0, 0) = 20;
  ROI_corners(0, 1) = 340;
  ROI_corners(0, 2) = 1;
  ROI_corners(1, 0) = 20;
  ROI_corners(1, 1) = 170;
  ROI_corners(1, 2) = 1;
  ROI_corners(2, 0) = 555;
  ROI_corners(2, 1) = 340;
  ROI_corners(2, 2) = 1;
  ROI_corners(3, 0) = 555;
  ROI_corners(3, 1) = 170;
  ROI_corners(3, 2) = 1;

  float lambda = 800.0f;

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

  IntrinsicMatrix params;
  params(0, 0) = 1202.806884765625;
  params(0, 1) = 0.0;
  params(0, 2) = 1275.52734375;

  params(1, 0) = 0.0;
  params(1, 1) = 1202.2362060546875;
  params(1, 2) = 733.5911254882812;
  params(2, 0) = 0.0;
  params(2, 1) = 0.0;
  params(2, 0) = 1.0;
  IntrinsicMatrix back_to_image_coords_matrix = params.inverse() * lambda;

  std::cout << ROI_corners * back_to_image_coords_matrix << std::endl;

  return 0;
}
