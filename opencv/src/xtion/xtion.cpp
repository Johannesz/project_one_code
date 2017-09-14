


#include <openni2/OpenNI.h>
#include <opencv2/opencv.hpp>


using namespace openni;

int main()
{
    OpenNI::initialize();
    puts( "xtion initialization..." );
    Device device;
    if ( device.open( openni::ANY_DEVICE ) != 0 )
    {
        puts( "xtion not found !" );
        return -1;
    }
    puts( "xtion opened" );
    VideoStream depth, color;
    color.create( device, SENSOR_COLOR );
    color.start();
    puts( "Camera ok" );
    depth.create( device, SENSOR_DEPTH );
    depth.start();
    puts( "Depth sensor ok" );
    VideoMode paramvideo;
    paramvideo.setResolution( 640, 480 );
    paramvideo.setFps( 30 );
    paramvideo.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );
    depth.setVideoMode( paramvideo );
    paramvideo.setPixelFormat( PIXEL_FORMAT_RGB888 );
    color.setVideoMode( paramvideo );
    puts( "hoffentlich" );

    // If the depth/color synchronisation is not necessary, start is faster :
    device.setDepthColorSyncEnabled( false );

    // Otherwise, the streams can be synchronized with a reception in the order of our choice :
    //device.setDepthColorSyncEnabled( true );
    //device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

    VideoStream** stream = new VideoStream*[2];
    stream[0] = &depth;
    stream[1] = &color;
    puts( "xtion initialization completed" );


    if ( device.getSensorInfo( SENSOR_DEPTH ) != NULL )
    {


       VideoFrameRef depthFrame, colorFrame;
       puts( "hier" );
       cv::Mat colorcv( cv::Size( 640, 480 ), CV_8UC3 );
        cv::Mat depthcv( cv::Size( 640, 480 ), CV_16UC1 );

        cv::namedWindow( "RGB", CV_WINDOW_AUTOSIZE );
        cv::namedWindow( "Depth", CV_WINDOW_AUTOSIZE );

        int changedIndex;
        while( device.isValid() )
        {
            OpenNI::waitForAnyStream( stream, 2, &changedIndex );
            switch ( changedIndex )
            {
                case 0:
                    depth.readFrame( &depthFrame );

                    if ( depthFrame.isValid() )
                    {
                        depthcv.data = (uchar*) depthFrame.getData();
                        cv::imshow( "Depth", depthcv );
                    }
                    break;

                case 1:
                    color.readFrame( &colorFrame );

                    if ( colorFrame.isValid() )
                    {
                        colorcv.data = (uchar*) colorFrame.getData();
                        cv::cvtColor( colorcv, colorcv, CV_BGR2RGB );
                        cv::imshow( "RGB", colorcv );
                    }
                    break;

                default:
                    puts( "Error retrieving a stream" );
            }
            cv::waitKey( 1 );
        }

        cv::destroyWindow( "RGB" );
        cv::destroyWindow( "Depth" );
    }
    depth.stop();
    depth.destroy();
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();
}
