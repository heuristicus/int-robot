package ex1;

import geometry_msgs.Twist;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.LaserScan;

/**
 * Moves the robot forward up to a set distance away from the obstacle
 * in front. Uses 20 laser readings from the centre of the laser's view.
 */
public class Avoider extends AbstractNodeMain {

    public static final String DASHES = "---------------------------------";//Debugging

    public static final double MAX_SPEED = 0.5f;
    public static final float SAFE_DISTANCE = 0.5f; // In metres
    public static final int RANGES_CHECKED = 20; // Check this many degrees
    public static final double EPSILON = 0.05;

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
                float avg = averageCentralReading(scan);
                System.out.println("AverageCentralReading: " + avg);
                if (avg > SAFE_DISTANCE + MAX_SPEED) {
                    System.out.println("SAFE max");
                    moveForward(MAX_SPEED);
                } else {
                    System.out.printf("publishing to twist:  " + (avg - SAFE_DISTANCE));
                    moveForward(0);

                }
            }
        });
    }

    public static float averageCentralReading(LaserScan scan)
            throws IllegalStateException {
        float[] ranges = scan.getRanges();
        if (ranges.length < RANGES_CHECKED) {
            throw new IllegalStateException("Oh noez, not enough angles");
        }

        int middleAngle = ranges.length/2;
        int startAngle = middleAngle-RANGES_CHECKED/2;
        int endAngle = middleAngle+RANGES_CHECKED/2;

        float totalRange = 0;
        for (int i = startAngle; i < endAngle; i++) {
            totalRange += ranges[i];
        }
        float averageRange = totalRange / RANGES_CHECKED;
        return averageRange;
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
}
