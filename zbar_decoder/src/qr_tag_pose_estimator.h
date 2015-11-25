#ifndef QRTAGPOSEESTIMATOR
#define QRTAGPOSEESTIMATOR

#include <iostream>
#include <zbar.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ros/ros.h"

using namespace std;
using namespace cv;
using namespace zbar;

class QrTagPoseEstimator
{
public:
  QrTagPoseEstimator() : tag_points_(4), cross_point_offset(32.5e-3f) {}
  void init(cv::Mat camera_matrix, cv::Mat dist_coeffs, double tag_width)
  {
    camera_matrix_ = camera_matrix;
    dist_coeffs_ = dist_coeffs;
    tag_points_[0] = cv::Point3f(0.f,0.f,0.f);
    tag_points_[2] = cv::Point3f(tag_width,tag_width,0.f);
    tag_points_[1] = cv::Point3f(tag_width,0.f,0.f);
    tag_points_[3] = cv::Point3f(0.f,tag_width,0.f);
  }
  void getPose(const zbar::Symbol &tag_sym, Mat &R, Mat &position)
  {
    vector<Point2f> image_points(4);
    for (int i = 0; i < 4; ++i) {
      image_points[i] = Point2f(tag_sym.get_location_x(i), tag_sym.get_location_y(i));
    }
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::solvePnP(tag_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec, CV_ITERATIVE);

    Rodrigues(rvec, R);

    Mat_<double> cam_to_marker_tf = cv::Mat::zeros(4,4,CV_64F);
    cam_to_marker_tf <<
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),tvec.at<double>(1),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2);
    cam_to_marker_tf(3,3) = 1;
    double cross_point_offset(32.5e-3f);
    cv::Mat_<double> marker_to_cross_tf = cv::Mat::eye(4,4,CV_64F);
    marker_to_cross_tf(0,3) = -cross_point_offset;
    marker_to_cross_tf(1,3) = -cross_point_offset;
    Mat_<double> cam_to_cross = cam_to_marker_tf * marker_to_cross_tf;
    position = cam_to_cross(cv::Range(0,3),cv::Range(3,4));
  }
#ifndef NDEBUG
  void showPoseEst(const zbar::Symbol &tag_sym, Mat &R, Mat &position, const Mat &qr_tag_im)
  {
    vector<Point2f> image_points(4);
    for (int i = 0; i < 4; ++i) {
      image_points[i] = Point2f(tag_sym.get_location_x(i), tag_sym.get_location_y(i));
    }
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::solvePnP(tag_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec, CV_ITERATIVE);
    Rodrigues(rvec, R);
    position = tvec;

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(tag_points_, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);
    // Show reprojection error
    Mat show_im;
    cvtColor(qr_tag_im, show_im,CV_GRAY2RGB);
    for(unsigned int i = 0; i < projected_points.size(); ++i)
    {
      std::cout << "Image point: " << image_points[i] << " Projected to " << projected_points[i] << std::endl;
      circle(show_im, image_points[i], 2, Scalar( 0, 255, 0 ), 1, 8 );
      circle(show_im, projected_points[i], 3, Scalar( 0, 0, 255 ), 1, 8 );
    }
    imshow("Image",show_im);
    waitKey();
    // display axis
    Mat axis_im;
    cvtColor(qr_tag_im, axis_im,CV_GRAY2RGB);
    double axis_length = 20e-3f;
    vector<Point3f> axis_pts(4);
    axis_pts[0] = Point3f(0,0,0);
    axis_pts[1] = Point3f(axis_length,0,0);
    axis_pts[2] = Point3f(0,axis_length,0);
    axis_pts[3] = Point3f(0,0,axis_length);
    std::vector<cv::Point2f> projectedAxisPoints;
    cv::projectPoints(axis_pts, rvec, tvec, camera_matrix_, dist_coeffs_, projectedAxisPoints);
    line(axis_im,projectedAxisPoints[0], projectedAxisPoints[1], Scalar( 0, 0, 255 ), 3, 8 );
    line(axis_im,projectedAxisPoints[0], projectedAxisPoints[2], Scalar( 0, 255, 0 ), 3, 8 );
    line(axis_im,projectedAxisPoints[0], projectedAxisPoints[3], Scalar( 255, 0, 0), 3, 8 );
    imshow("Image",axis_im);
    waitKey();
  }
#endif
private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  vector<cv::Point3f> tag_points_;
  double cross_point_offset;
};

#endif // QRTAGPOSEESTIMATOR

