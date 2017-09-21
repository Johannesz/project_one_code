#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

int thresh = 200;
int max_thresh = 255;
char* source_window = "Source image";
char* corners_window = "Corners detected";
cv::Mat src;
void cornerHarris_demo( int, void* );



int main(int argc, char **argv) {

	char* imageName = argv[1];
	cv::Mat image;
	image = imread(imageName, cv::IMREAD_COLOR);
	if (argc != 2 || !image.data) {
		printf(" No image data \n ");
		return -1;
	}

	// Alternativen evlt. noch testen find contours, template matching



	cv::Mat blue_filter;
   cv::inRange(image, cv::Scalar(200, 0, 0), cv::Scalar(255, 255, 255), blue_filter);

   cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                        cv::Size( 19, 19 ),
                        cv::Point( 8, 8 ) );
   cv::Mat erosion;
   cv::erode( blue_filter, erosion, element );

   cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
   cv::createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
     imshow( source_window, src );
     cornerHarris_demo( 0, 0 );


	//Fenster


	cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
	cv::imshow("color", blue_filter);

	cv::namedWindow("erosion", cv::WINDOW_AUTOSIZE);
	cv::imshow("erosion", erosion);

	cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
	cv::imshow("original", image);

	cv::waitKey(0);

	return 0;
}

/* @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros( src.size(), CV_32FC1 );
  int blockSize = 4;
  int apertureSize = 3;
  double k = 0.16;
  cv::cornerHarris( src, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               cv::circle( dst_norm_scaled, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
              }
          }
     }
  cv::namedWindow( corners_window, cv::WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}
