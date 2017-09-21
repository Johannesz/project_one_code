#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
using namespace cv;
using namespace std;
Mat src, src_gray;
int thresh = 200;
int max_thresh = 255;
char* source_window = "Source image";
char* corners_window = "Corners detected";
void cornerHarris_demo( int, void* );
/* @function main */
int main( int argc, char** argv )
{
  src = imread( argv[1], 1 );
  //cv::medianBlur(src, src, 7);
  cv::inRange(src, cv::Scalar(210, 0, 0), cv::Scalar(255, 255, 255), src);

  cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                       cv::Size( 19, 19 ),
                       cv::Point( 8, 8 ) );
  cv::Mat erosion;
  cv::erode( src, src, element );



  //cvtColor( src, src_gray, COLOR_BGR2GRAY );
  namedWindow( source_window, WINDOW_AUTOSIZE );
  createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
  imshow( source_window, src );
  cornerHarris_demo( 0, 0 );
  waitKey(0);
  return(0);
}
/* @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{
  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );
  int blockSize = 4;
  int apertureSize = 3;
  double k = 0.16;
  cornerHarris( src, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
  namedWindow( corners_window, WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}

