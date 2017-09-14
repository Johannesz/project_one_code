#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>


int main(int argc, char **argv)
{

   float distanceMiddleX = 0.0; //distance to middle pixel
   float distanceMiddleY = 0.0; //distance to middle pixel
   float distanceSum = 0.0;
   int circleForDepthZ = 0; //which found circle will be used for depth calculation
   int resolutionX = 640/2; //camera resolution
   int resolutionY = 480/2; //camera resolution
   float max = 5000;    // high number for iteration
   int focal_length_z [3] = { 810, 825, 870}; // measured focal_length in mm for different distances for depth measurment
   float depthZ = 0.0; //depth distance z to pipe
   float ros_cam_calibration [4] = { 813, 757, 291, 249 }; // fx, fy, cpX, cpY - out of calibration file

   float xCord = 0.0;   //distance from camera center to pipe center in mm
   float yCord = 0.0;   //distance from camera center to pipe center in mm

    char* imageName = argv[1];
    cv::Mat image;
    image = imread( imageName, cv::IMREAD_COLOR );
    if( argc != 2 || !image.data )
    {
      printf( " No image data \n " );
      return -1;
    }

	cv::Mat orig_image = image.clone();

	cv::medianBlur(image, image, 3);

	// Convert input image to HSV
	cv::Mat hsv_image;
	cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

	// Threshold the HSV image, keep only the white pixels
	cv::Mat lower_white_hue_range;
	//
	cv::inRange(hsv_image, cv::Scalar(0, 0, (255/100*79)), cv::Scalar(180, (255/100*31), 255), lower_white_hue_range);
	cv::GaussianBlur(lower_white_hue_range, lower_white_hue_range, cv::Size(9, 9), 2, 2);

	// Use the Hough transform to detect circles in the combined threshold image
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(lower_white_hue_range, circles, CV_HOUGH_GRADIENT, 1, lower_white_hue_range.rows/8, 60, 60 );

	for (auto const& circle : circles)
	{
	    std::cout << circle << std::endl;
	}

	// Loop over all detected circles and outline them on the original image
	if(circles.size() == 0) std::exit(-1);
	for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
		cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
		int radius = std::round(circles[current_circle][2]);

		cv::circle(orig_image, center, radius/50, cv::Scalar(0, 255, 0), 5);
		cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 2);

	}

   // Search for circle which has the minimum distance to the center of the taken picture
   for (size_t current_circle = 0; current_circle < circles.size();
         ++current_circle) {

      distanceMiddleX = abs(resolutionX - circles[current_circle][0]);
      distanceMiddleY = abs(resolutionY - circles[current_circle][1]);

      distanceSum = distanceMiddleX + distanceMiddleY;

      if (distanceSum < max) {
         max = distanceSum;
         circleForDepthZ = current_circle;
      }

   }

   std::cout << "Circle number: " << circleForDepthZ << "  with the Center: "
         << circles[circleForDepthZ][0] << "  " << circles[circleForDepthZ][1]
         << std::endl;

   // The depth calculation will later on replaced, because we get the depth to ground from the 3D Camera out of the first localisation.
   // calculate depth depending on the amount of pixels of the circel diameter - values are for Sony Playstation Eye Camera

   if (circles[circleForDepthZ][2] * 2 < 100) {
      depthZ = (focal_length_z[0] * 60) / (circles[circleForDepthZ][2] * 2);

   } else if ((circles[circleForDepthZ][2] * 2 > 100)
         && (circles[circleForDepthZ][2] * 2 < 200)) {
      depthZ = (focal_length_z[1] * 60) / (circles[circleForDepthZ][2] * 2);
   } else {
      depthZ = (focal_length_z[2] * 60) / (circles[circleForDepthZ][2] * 2);
   }

   std::cout << "Abstand z zum Rohr:" << depthZ << std::endl;


   // Calculate X, Y Position of Cylinder
   /*
   // First get data out of the calibration file:
   std::string filename = "ros_calib_ps_eye.yaml";
   cv::FileStorage fs;
   fs.open(filename, cv::FileStorage::READ);

   cv::Mat camera_matrix_code;

   fs["camera_matrix"] >> camera_matrix_code;
   std::cout << "camera matrix: " << camera_matrix_code << std::endl;
     fs.release();
   */

   for (size_t current_circle = 0; current_circle < circles.size();
            ++current_circle)
   {
      xCord = -((circles[current_circle][0] - ros_cam_calibration[2]) / ros_cam_calibration[0]) * depthZ;
      yCord = -((circles[current_circle][1] - ros_cam_calibration[3]) / ros_cam_calibration[1]) * depthZ;

      std::cout << " Zirkel Nr." << current_circle << " : " << " X: " << xCord << " Y: " << yCord << std::endl << std::endl;

   }




	// Show images
	cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Threshold lower image", lower_white_hue_range);
   cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
   cv::imshow("Original", orig_image);


	cv::waitKey(0);

	
	return 0;
}
