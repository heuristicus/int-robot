/**
package ex4;

import com.googlecode.javacv.cpp.opencv_objdetect.CvHaarClassifierCascade;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_highgui;


import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.cvSaveImage;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_BGR2GRAY;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvCvtColor;
import static com.googlecode.javacv.cpp.opencv_objdetect.cvHaarDetectObjects;

public class FaceDetectToFile{
  // The cascade definition to be used for detection.
  //private static final String CASCADE_FILE =
    //"/data/private/robot/Downloads/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_default.xml";

  public static void main(String[] args) throws Exception {
      String cascadeFile = args[0];

    // Load the original image.
    opencv_core.IplImage originalImage = opencv_highgui.cvLoadImage(args[1], 1);

    // We need a grayscale image in order to do the recognition, so we
    // create a new image of the same size as the original one.
    opencv_core.IplImage grayImage = opencv_core.IplImage.create(
        originalImage.width(),
        originalImage.height(),
        opencv_core.IPL_DEPTH_8U, 1);

    // We convert the original image to grayscale.
    cvCvtColor(originalImage, grayImage, CV_BGR2GRAY);

    opencv_core.CvMemStorage storage = opencv_core.CvMemStorage.create();

    // We instantiate a classifier cascade to be used for detection,
    // using the cascade definition.
    CvHaarClassifierCascade cascade = new CvHaarClassifierCascade(cvLoad(cascadeFile));

    // We detect the faces.
    opencv_core.CvSeq faces = cvHaarDetectObjects(
        grayImage, cascade, storage, 1.1, 1, 0);

    // We iterate over the discovered faces and draw yellow rectangles
    // around them.
    for (int i = 0; i < faces.total(); i++) {
      opencv_core.CvRect r = new opencv_core.CvRect(cvGetSeqElem(faces, i));
      cvRectangle(originalImage, cvPoint(r.x(), r.y()),
      cvPoint(r.x() + r.width(), r.y() + r.height()),
          opencv_core.CvScalar.YELLOW, 1, CV_AA, 0);

        System.out.println("Found face at x: "+r.x()+" y: "+r.y() + " to x: " + (r.x()+r.width())+" y: "+(r.y()+r.height()));
    }

    // Save the image to a new file.
    cvSaveImage(args[2], originalImage);
  }
}
*/