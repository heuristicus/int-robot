package ex1;

import geometry_msgs.Twist;
import java.util.Arrays;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.LaserScan;

public class Avoider2 extends AbstractNodeMain {

    public static final String DASHES = "---------------------------------";//Debugging

    public static final double MAX_SPEED = 0.5f;
    public static final float SAFE_DISTANCE = 0.5f; // In metres
    public static final int SECTORS_CHECKED = 21; // Check this many sectors
    public static final double EPSILON = 0.05;
    public static final double DEFAULT_ROTATION = 0.087266389; // 5 degrees
    public static final float DEFAULT_REPLACEMENT_FOR_ZERO = 7.0f;
    public static final float IGNORE_THRESHOLD = 0.01f;

    private Publisher<Twist> pub;
    private Subscriber<LaserScan> laser;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("avoider");
    }

    @Override
    public void onStart(ConnectedNode node) {
        pub = node.newPublisher("cmd_vel", Twist._TYPE);
        laser = node.newSubscriber("base_scan", LaserScan._TYPE);

        System.out.println("OnStart called "+DASHES);

        laser.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan scan) {
                System.out.println(scan.getRanges().length);

                float[][] readings = LaserUtil.getSectors(SECTORS_CHECKED, scan);
                float[] avgs = LaserUtil.averageOfEachSector(readings);
                float minAvg = LaserUtil.minReading(avgs, DEFAULT_REPLACEMENT_FOR_ZERO, IGNORE_THRESHOLD);

                System.out.println("Averages: ");
                for (int i= 0; i < avgs.length; i++) {
                    System.out.println(avgs[i]);
                }
                System.out.println("");

                System.out.println("MinAvg: " + minAvg);
                if (minAvg > SAFE_DISTANCE + MAX_SPEED) {
                    System.out.println("SAFE max");
                    moveForward(MAX_SPEED);
                } else {
                    System.out.println("Rotating: "+DEFAULT_ROTATION);
                    //moveForward(avg - SAFE_DISTANCE);
                    rotate(DEFAULT_ROTATION);
                }
            }
        });
    }

    public boolean moveForward(double distance) {
        return moveForward(distance, EPSILON);
    }

    public boolean moveForward(double distance, double epsilon) {
        if (Math.abs(distance) < epsilon) {
            return false;
        }
        Twist twist = pub.newMessage();
        twist.getLinear().setX(distance);
        pub.publish(twist);
        return true;
    }

    /** Rotate this angle in radians, clockwise. */
    public void rotate(double theta) {
        Twist twist = pub.newMessage();
        twist.getAngular().setZ(-theta);
        pub.publish(twist);
    }
}
