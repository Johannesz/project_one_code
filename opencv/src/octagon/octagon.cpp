#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

int main(int argc, char **argv) {

	char* imageName = argv[1];
	cv::Mat image;
	image = imread(imageName, cv::IMREAD_GRAYSCALE);
	if (argc != 2 || !image.data) {
		printf(" No image data \n ");
		return -1;
	}

	//histogram analysis
	cv::Mat histo;
	equalizeHist(image, histo);

	//Gausian Blur
	cv::Mat gauss_blur;
	cv::GaussianBlur(histo, gauss_blur, cv::Size(3, 3), 0);

	cv::Mat thres;


	cv::Canny(gauss_blur, thres, 330, 400, 3);

	// Throshold for creating 0 - 1 image
	//adaptiveThreshold(gauss_blur, thres, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
	//		CV_THRESH_BINARY, 75, 0);
	// cv::Mat bit;
	// cv::bitwise_not(thres, bit);

	// Hough line detection
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(thres, lines, 1, (CV_PI / 180)*1, 5, 20, 10);
	if (lines.size() == 0)
		std::exit(-1);
	for (size_t i = 0; i < lines.size(); i++) {
		cv::Vec4i l = lines[i];
		line(image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 0, 255), 3, CV_AA);
	}



	//Fenster


	//cv::namedWindow("histo", cv::WINDOW_AUTOSIZE);
	//cv::imshow("histo", histo);

	//cv::namedWindow("gauss_blur", cv::WINDOW_AUTOSIZE);
	//cv::imshow("gauss_blur", gauss_blur);



	cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
	cv::imshow("thres", thres);

	cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
	cv::imshow("original", image);

	cv::waitKey(0);

	return 0;
}
