/*
package ex4;

import com.googlecode.javacv.cpp.opencv_objdetect.CvHaarClassifierCascade;
import com.googlecode.javacv.cpp.opencv_core;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_BGR2GRAY;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvCvtColor;
import static com.googlecode.javacv.cpp.opencv_objdetect.cvHaarDetectObjects;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import sensor_msgs.CompressedImage;

public class FaceDetect extends AbstractNodeMain {

    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;

    // The cascade definition to be used for detection.
    private static final String CASCADE_FILE =
            "/data/private/robot/Downloads/OpenCV-2.4.3/data/haarcascades/"
            + "haarcascade_frontalface_default.xml";

    FaceDetectGUI.Panel panel;
    FaceDetectGUI.Window window;

    Subscriber<CompressedImage> imageSub;

    @Override
    public void onStart(ConnectedNode connectedNode) {
        imageSub = connectedNode.newSubscriber("out/compressed", CompressedImage._TYPE); // WRONG!!!!

        panel = new FaceDetectGUI.Panel(IMAGE_WIDTH, IMAGE_HEIGHT);
        window = new FaceDetectGUI.Window(panel, IMAGE_WIDTH, IMAGE_HEIGHT);
        window.setVisible(true);

        imageSub.addMessageListener(new MessageListener<CompressedImage>() {
            @Override
            public void onNewMessage(CompressedImage compImg) {
                BufferedImage bufImg = new BufferedImage(
                        IMAGE_WIDTH, IMAGE_HEIGHT, BufferedImage.TYPE_3BYTE_BGR);

//                int[] rgb = convertToIntArray(compImg.getData().array());
                byte[] bytes = compImg.getData().array();
                int[] bgr = new int[bytes.length];
                System.arraycopy(bytes, 0, bgr, 0, bytes.length);
                bufImg.setRGB(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, bgr, 0, IMAGE_WIDTH); // , IMAGE_WIDTH*3);

                IplImage originalImage = IplImage.createFrom(bufImg);

                // We need a grayscale image in order to do the recognition, so we
                // create a new image of the same size as the original one.
                IplImage grayImage = IplImage.create(
                        originalImage.width(),
                        originalImage.height(),
                        opencv_core.IPL_DEPTH_8U, 1);

                // We convert the original image to grayscale.
                cvCvtColor(originalImage, grayImage, CV_BGR2GRAY);

                CvMemStorage storage = CvMemStorage.create();

                // We instantiate a classifier cascade to be used for detection,
                // using the cascade definition.
                CvHaarClassifierCascade cascade = new CvHaarClassifierCascade(cvLoad(CASCADE_FILE));

                // We detect the faces.
                CvSeq faces = cvHaarDetectObjects(
                        grayImage, cascade, storage, 1.1, 1, 0);

                Graphics2D g = bufImg.createGraphics();

                // We iterate over the discovered faces and draw yellow rectangles
                // around them.
                for (int i = 0; i < faces.total(); i++) {
                    CvRect r = new CvRect(cvGetSeqElem(faces, i));

//                    cvRectangle(originalImage, cvPoint(r.x(), r.y()),
//                            cvPoint(r.x() + r.width(), r.y() + r.height()),
//                            opencv_core.CvScalar.YELLOW, 1, CV_AA, 0);

                    System.out.println("Found face at x: " + r.x() + " y: " + r.y() + 
                            " to x: " + (r.x() + r.width()) + 
                            " y: " + (r.y() + r.height()));
                    g.drawRect(r.x(), r.y(), r.width(), r.height());
                }

                // Display image
                panel.getGraphics().drawImage(bufImg, 0, 0, null);
            }
        });
    }

    /** Given an array of bytes which contains triplets of bytes which belong
     * in one int, this method combines them into an array of ints a third of
     * the size of the byte array. This is done by appending triplets of bytes
     * into ints

    public static int[] convertToIntArray(byte[] bytes) {
        if (bytes.length % 3 != 0) {
            throw new IllegalArgumentException("Byte array is not divisible by 3");
        }
        int[] ints = new int[bytes.length / 3];
        int j;

        for (int i = 0; i < ints.length; i++) {
            j = i * 3;
            ints[i] = combineBytes(
                    bytes[j],
                    bytes[j + 1],
                    bytes[j + 2]);
        }

        return ints;
    }

    /** Given three bytes, combines into an int (by appending bytes).
     * This is for combining rgb values from bytes into one RGB int value. 
    public static int combineBytes(byte a, byte b, byte c) {
        return (a << 16) + (b << 8) + c;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("FaceDetect");
    }
}
*/