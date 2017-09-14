#include "opencv2/opencv.hpp"
#include <iostream>
 
using namespace std;
using namespace cv;
 
int main(){
 
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap(1);
    
  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
     
  while(1){
 
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
 
    // Display the resulting frame
    imshow( "Frame", frame );

    imwrite("saved.jpg", frame);
    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}

/*
#include <opencv2/opencv.hpp>
using namespace cv;
int main( int argc, char** argv )
{
 char* imageName = argv[1];
 Mat image;
 image = imread( imageName, 1 );
 if( argc != 2 || !image.data )
 {
   printf( " No image data \n " );
   return -1;
 }
 Mat gray_image;
 cvtColor( image, gray_image, COLOR_BGR2GRAY );
 imwrite( "../../images/Gray_Image.jpg", gray_image );
 namedWindow( imageName, WINDOW_AUTOSIZE );
 namedWindow( "Gray image", WINDOW_AUTOSIZE );
 imshow( imageName, image );
 imshow( "Gray image", gray_image );
 waitKey(0);
 return 0;
}
*/
