#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // For isnan
#include <math.h>
//#include <cv_bridge/CvBridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv_bridge;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char DEPTHWIN[] = "Depth";
static const char DASH[] = "-----------------------"; // For cout
static const float THRESHOLD_DISTANCE = 3.5f;
static const float DEPTH_TOLERANCE = 0.1f; // Percentage

/* The part of a rectangle which represents a face which we will
   use to extract the depth (i.e. the depth of the face will 
   be represented by this portion). E.g. 0.2 is the central 20% width
   and central 20% height of the rectangle */
static const double FACE_SIGNIFICANT_PORTION = 0.2;

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
	CvImagePtr lastDepthReading;
	bool depthReadingUsed;
	CvPoint segpt1;
	CvPoint segpt2;
  
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
	depthReadingUsed = true; // Wait for depth reading to be filled the first time

    cv::namedWindow(WINDOW); // RGB window
    cv::namedWindow(DEPTHWIN); // DEPTH WIN

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
	cv::destroyWindow(DEPTHWIN);
	if(pCascade) cvReleaseHaarClassifierCascade(&pCascade);
	if(pStorage) cvReleaseMemStorage(&pStorage);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	if (depthReadingUsed) {
		return; // We have used this depth reading. Wait for another
	}
	cv_bridge::CvImage bridge;

    /*try{
        img = bridge.imgMsgToCv(msg, enc::BGR8);
    } catch(sensor_msgs::CvBridgeException error) {
        ROS_ERROR("Failed to convert sensor_msgs::Image to img_t with error [%d]", error);
        return 0;
    }*/

    cout << "RGB CallBack " << rgbCount++ << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); // Colour
//		cv_ptr = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8); // Blackandwhite
//		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8); // IR
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	// detect faces in image
	IplImage iplImg = cv_ptr->image; // Convert
	pFaceRectSeq = cvHaarDetectObjects
		( &(iplImg) , pCascade, pStorage,
		1.1,                       // increase search scale by 10% each pass
		3,                         // drop groups of fewer than three detections
		CV_HAAR_DO_CANNY_PRUNING/*,  // skip regions unlikely to contain a face
		cvSize(20,20),              // use XML default for smallest search scale
		cvSize(100,100)*/
	);

    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

	// draw a rectangular outline around each detection
	for(i=0;i<(pFaceRectSeq? pFaceRectSeq->total:0); i++ ) {
		CvRect* r = (CvRect*)cvGetSeqElem(pFaceRectSeq, i);
	   
		cout << "Calling validface" << endl;
		CvScalar rectColor;
		if (validFace(lastDepthReading, r)) {
			cout << "Valid face detected " << i << endl;
			rectColor = CV_RGB(0,255,0); // Green
		} else {
			rectColor = CV_RGB(255,0,0); // Red
		}

		CvScalar sColour = CV_RGB(0,0,255); // Blue
		cv::rectangle(cv_ptr->image, segpt1, segpt2, sColour, 3, 4, 0); // DEBUG ONLY -- Draw segment rect

		CvPoint pt1 = { r->x, r->y };
		CvPoint pt2 = { r->x + r->width, r->y + r->height };
		cv::rectangle(cv_ptr->image, pt1, pt2, rectColor, 3, 4, 0);
	}

	cout << DASH << DASH << DASH << DASH << endl;

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    //image_pub_.publish(cv_ptr->toImageMsg());

	// This reading is now expired, wait for another before entering this function again
	depthReadingUsed = true;
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    cout << "Depth Callback " << depthCount++ << endl;
    try {
		lastDepthReading = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1); // Depth
		CvScalar sColour = CV_RGB(0,0,255); // Blue
		cv::rectangle(lastDepthReading->image, segpt1, segpt2, sColour, 3, 4, 0); // DEBUG ONLY -- Draw segment rect

		depthReadingUsed = false;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::imshow(DEPTHWIN, lastDepthReading->image);
    cv::waitKey(3);
  }

  // Gives the start and end pixels of the segment in the 
  // centre of the rectangle (only along one axis)
  void getStartAndEndPixels(int totalPixels, int segment, 
							int* startPixel, int* endPixel) {
	*startPixel = (totalPixels-segment)/2;
	*endPixel = *startPixel + segment;
  }

  bool validFace(cv_bridge::CvImagePtr &depth, CvRect* rect) {
	/*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

	cv::Mat img = depth->image;

	cout << "Depth img width: " << img.cols << " height: " << img.rows << endl;

	int startX;
	int endX;
	getStartAndEndPixels(rect->width, rect->width * FACE_SIGNIFICANT_PORTION, &startX, &endX);
	cout << "Initial startX: " << startX << " endX: " << endX << endl;
	startX += rect->x;
	endX += rect->x;

	int startY;
	int endY;
	getStartAndEndPixels(rect->width, rect->width * FACE_SIGNIFICANT_PORTION, &startY, &endY);
	cout << "Initial startY: " << startY << " endY: " << endY << endl;
	startY += rect->y;
	endY += rect->y;

	// Debugging -- for drawing the segment
	segpt1 = { startX, startY };
	segpt2 = { endX, endY };

	float sum = 0;
	int pixels = 0;
	
	std::vector<float> vals;

	for (int y = startY; y <= endY; y++) {
		for (int x = startX; x <= endX; x++) {
			//pixelIndex = width*y + x;
			//sum += img.data[pixelIndex];
			float val = img.at<float>(y, x);
			if (! isnan(val) && isfinite(val)) {
				sum += val; // DELETEME
				vals.push_back(val);
				//cout << val << ", ";
				pixels++;
			}
		}
		//cout << endl;
	}

	// DEBUGGING
	for (int y = rect->y; y <= rect->y+rect->width; y++) {
		for (int x = rect->x; x <= rect->x+rect->width; x++) {
			//pixelIndex = width*y + x;
			//sum += img.data[pixelIndex];
			float val = img.at<float>(y, x);
			if (! isnan(val) && isfinite(val)) {
				sum += val; // DELETEME
				vals.push_back(val);
				pixels++;
			}
			cout << val << ", ";
		}
		cout << endl;
	}

	double meanAvg = sum / pixels; // Could be nan
	cout << "Rect at x: " << startX << " y: " << startY
		 << " width: " << endX-startX << " height: " << endY-startY << endl;
	cout << "Face rect is (x,y):   (" << rect->x << "," << rect->y << ")   to   "
		 << "(" << rect->x + rect->width << "," << rect->y + rect->height << ") " 
		 << "width: " << rect->width << " height: " << rect->height << endl;

	float median;
	if (vals.size() > 0) {
		cout << "vals.size():" << vals.size() << endl;
		size_t n = vals.size() / 2;
		nth_element(vals.begin(), vals.begin()+n, vals.end());
		median = vals[n];
		cout << "MEDIAN: " << median << endl;
		if (median > THRESHOLD_DISTANCE) {
			cout << "Throwing away. Median > " << THRESHOLD_DISTANCE << endl;
			return false;
		} else {
			const float expected = expectedDepth(rect->width);
			const float tolerance = expected * DEPTH_TOLERANCE;
			const float diff = abs(median-expected);
			cout << "EXPECTED: " << expected << "   DIFF: " << diff 
				 << "   TOLERANCE: " << tolerance << endl;
			if (diff > tolerance) {
				cout << "Rejecting. Doesn't match observation model" << endl;
				return false; // Not within some percentage of expected depth
			}
		}
		return true;
	} else {
		cout << "Rejected. All NaNs (too close to camera)" << endl;
		return false;
	}
  }

  float expectedDepth(float rectWidth) {
	const float coefficient = 128.33f; // Obtained from an observation model
	const float power = -1.085468f;	   // Obtained from an observation model
	float depth = coefficient * pow(rectWidth, power);
	return depth;
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

