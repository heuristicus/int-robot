#include "cv.h"
#include "highgui.h"
#include <stdlib.h>

//const char* cascade_name = "/data/private/robot/Downloads/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_default.xml";

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class RectangulatorLive
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;
	image_transport::Subscriber rgb_sub_;
	image_transport::Subscriber depth_sub_;

	// declarations
	CvHaarClassifierCascade * pCascade;  // the face detector
	CvMemStorage * pStorage;        // expandable memory buffer
	CvSeq * pFaceRectSeq;               // list of detected faces
	int i;
	int rgbCount;
	int depthCount;
	cv_bridge::CvImagePtr lastDepthReading;
  
public:
  RectangulatorLive()
    : it_(nh_)
  {
	cout << "STARTING" << endl;
    image_pub_ = it_.advertise("out", 1);
    //rgb_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &RectangulatorLive::imageCb, this); // Logitech cam
	rgb_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &RectangulatorLive::imageCb, this); // Kinect
	depth_sub_ = it_.subscribe("/camera/depth/image", 1, &RectangulatorLive::depthCb, this); // Depth
	rgbCount = 0;
	depthCount = 0;

    cv::namedWindow(WINDOW);

	char OPENCV_ROOT[] = "/data/private/robot/Downloads/OpenCV-2.4.3/";
	std::string cascadePath;
	cascadePath += OPENCV_ROOT;
	cascadePath += "/data/haarcascades/haarcascade_frontalface_default.xml";
	//cascadePath += "/data/haarcascades/haarcascade_frontalface_alt.xml";
	//cascadePath += "/data/haarcascades/haarcascade_frontalface_alt2.xml";
	//cascadePath += "/data/haarcascades/haarcascade_frontalface_alt_tree.xml";
	//cascadePath += "/data/haarcascades/haarcascade_profileface.xml";
	//cascadePath += "/data/haarcascades/haarcascade_upperbody.xml";

	// initializations
	pStorage = cvCreateMemStorage(0);
	pCascade = (CvHaarClassifierCascade *)cvLoad
	  ((cascadePath.c_str()),
	  0, 0, 0 );

	// validate that everything initialized properly
	if( !pStorage || !pCascade )
	{
	   printf("Initialization failed: %s \n",
	     (!pCascade)? "didn't load Haar cascade -- "
		          "make sure path is correct" :
	     "failed to allocate memory for data storage");
	   exit(-1);
	}
  }

  ~RectangulatorLive()
  {
    cv::destroyWindow(WINDOW);
	if(pCascade) cvReleaseHaarClassifierCascade(&pCascade);
	if(pStorage) cvReleaseMemStorage(&pStorage);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

	/*IplImage* img = 0;

    //std::string cv_encoding="bgr8";
    //sensor_msgs::CvBridge bridge;
	//cv_bridge::CvImage 
	cv_bridge::CvImage bridge;

    try{
        img = bridge.imgMsgToCv(msg, enc::BGR8);
    }catch(sensor_msgs::CvBridgeException error) {
        ROS_ERROR("Failed to convert sensor_msgs::Image to img_t with error [%d]", error);
        return 0;
    }*/

    cout << "CallBack " << rgbCount++ << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); // Colour
//		cv_ptr = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8); // Blackandwhite
//		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8); // IR
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	// detect faces in image
	/*pFaceRectSeq = cvHaarDetectObjects
	   (&(cv_ptr->image) , pCascade, pStorage,
	   1.1,                       // increase search scale by 10% each pass
	   3,                         // drop groups of fewer than three detections
	   CV_HAAR_DO_CANNY_PRUNING,  // skip regions unlikely to contain a face
	   cvSize(0,0),               // use XML default for smallest search scale
	   cvSize(100,100));              */

	IplImage iplImg = cv_ptr->image;

	pFaceRectSeq = cvHaarDetectObjects
	   ( &(iplImg) , pCascade, pStorage,
	   1.1,                       // increase search scale by 10% each pass
	   3,                         // drop groups of fewer than three detections
	   CV_HAAR_DO_CANNY_PRUNING/*,  // skip regions unlikely to contain a face
	   cvSize(20,20)              // use XML default for smallest search scale*/
	);

    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

	// draw a rectangular outline around each detection
	for(i=0;i<(pFaceRectSeq? pFaceRectSeq->total:0); i++ )
	{
	   CvRect * r = (CvRect*)cvGetSeqElem(pFaceRectSeq, i);
	   CvPoint pt1 = { r->x, r->y };
	   CvPoint pt2 = { r->x + r->width, r->y + r->height };
	   cv::rectangle(cv_ptr->image, pt1, pt2, CV_RGB(0,255,0), 3, 4, 0);
	}

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    cout << "Depth " << depthCount++ << endl;
    try
    {
//      lastDepthReading = cv_bridge::toCvCopy(msg, enc::BGR8); // Colour
		lastDepthReading = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1); // Depth
//		lastDepthReading = cv_bridge::toCvCopy(msg, enc::MONO8); // IR
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//Mat mat = cv_ptr->image;

    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

	// draw a rectangular outline around each detection
	/*for(i=0;i<(pFaceRectSeq? pFaceRectSeq->total:0); i++ )
	{
	   CvRect * r = (CvRect*)cvGetSeqElem(pFaceRectSeq, i);
	   CvPoint pt1 = { r->x, r->y };
	   CvPoint pt2 = { r->x + r->width, r->y + r->height };
	   cv::rectangle(cv_ptr->image, pt1, pt2, CV_RGB(0,255,0), 3, 4, 0);
	}*/

    //cv::imshow(WINDOW, lastDepthReading->image);
    //cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  RectangulatorLive ic;
  ros::spin();
  return 0;
}




/*int main ( int argc, const char* argv[] )
{

	// initializations
	IplImage * pInpImg = (argc > 1) ?
	   cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) : 0;
	

	// create a window to display detected faces
	cvNamedWindow("Haar Window", CV_WINDOW_AUTOSIZE);

	

	// display face detections
	cvShowImage("Haar Window", pInpImg);
	cvWaitKey(0);
	cvDestroyWindow("Haar Window");

	// clean up and release resources
	cvReleaseImage(&pInpImg);


	return 0;
}*/

