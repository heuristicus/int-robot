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
import util.LaserUtil;

public class Avoider3 extends AbstractNodeMain {

    public static final String DASHES = "---------------------------------";//Debugging

    public static final double MAX_SPEED = 0.6f;
    public static final float SAFE_DISTANCE = 0.5f; // In metres
    public static final int SECTORS_CHECKED = 21; // Check this many sectors
    public static final double EPSILON = 0.05;
    public static final double DEFAULT_ROTATION = 0.3f; // about 10 degrees
    public static final float IGNORE_THRESHOLD = 0.01f;

    private Publisher<Twist> pub;
    private Subscriber<LaserScan> laser;

    // Direction of obstacle (so rotate in opposite direction)
    private ObstacleDirection obstacleDirection = ObstacleDirection.UNSET;

    public enum ObstacleDirection {
        UNSET,
        RIGHT,
        LEFT;
    }

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
                System.out.println("Laser rangeMin is: " + scan.getRangeMin());
                System.out.println("Laser rangeMax is: " + scan.getRangeMax());

                float[][] readings = LaserUtil.getSectors(SECTORS_CHECKED, scan);
                //LaserUtil.printSectors(readings);
                float[] medians = LaserUtil.medianOfEachSector(readings);
                int minMedianPos = LaserUtil.minReadingPos(medians, IGNORE_THRESHOLD);
                // minMedianPos returns -1 if all sectors are trash or zero.
                float minMedian;
                minMedian = minMedianPos == -1 ? -1 : medians[minMedianPos];

                System.out.println("Medians: ");
                for (int i= 0; i < medians.length; i++) {
                    System.out.print(medians[i] + "\t");
                }
                System.out.println("");

                System.out.println("MinMedian: " + minMedian);
                // if the minimum median value is a safe distance away
                // or the method returning the median point returned an error value, move forwards.
                // the error value indicates that the values in the median array were trash values
                // or zero values, which indicate that the path ahead is likely to be clear.
                // any normal return value from the method indicates that there is something to be seen.
                if (minMedian > SAFE_DISTANCE + MAX_SPEED) {
                    System.out.println("SAFE: minMedian is " + minMedian + ", allowable distance: " + (SAFE_DISTANCE + MAX_SPEED));
                    obstacleDirection = ObstacleDirection.UNSET;
                    moveForward(MAX_SPEED);
                } else {
                    System.out.println("UNSAFE: minMedian is less than "
                            + (SAFE_DISTANCE + MAX_SPEED) + ". Rotating: " + DEFAULT_ROTATION);

                    if (obstacleDirection == ObstacleDirection.UNSET) {
                        // Which direction is the closest obstacle in?
                        // We +1 because minMedianPos returns 0-based array index
                        obstacleDirection = getDirectionOfObstacle(SECTORS_CHECKED, minMedianPos+1);
                        System.out.println("Obstacle detected in " + obstacleDirection 
                                + " direction. Will turn in opposite direction until clear.");
                    }
                    System.out.println("Continuing turn in " + obstacleDirection);
                    rotate(obstacleDirection, DEFAULT_ROTATION);
                }
            }
        });
    }

    public ObstacleDirection getDirectionOfObstacle(int numOfSectors, int sector) {
        // If we have the middle sector of an odd number of sectors,
        // the object is dead ahead. So compute a random direction. Lol.
        if (numOfSectors % 2 == 1 &&
                (sector == (numOfSectors/2) + 1)) {
            return Math.random() < 0.5 ? ObstacleDirection.LEFT : ObstacleDirection.RIGHT;
        }

        if (sector > (numOfSectors/2)) {
            return ObstacleDirection.RIGHT;
        } else {
            return ObstacleDirection.LEFT;
        }
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

    /** Turns clockwise in the given number of radians.
     * Positive goes clockwise. */
    public void rotate(double theta) {
        Twist twist = pub.newMessage();
        twist.getAngular().setZ(theta); // Don't think this needs to be minus (at least, it rotates the wrong way when using the other method)
        pub.publish(twist);
    }

    /** Rotates in the opposite direction to the nearest obstacle */
    public void rotate(ObstacleDirection obstacleDir, double theta) {
        // If obstacle is on the right, we turn anti-clockwise.
        // All other cases turn clockwise (including UNSET)
        theta = obstacleDir == ObstacleDirection.RIGHT ? -theta : theta;
        rotate(theta);
    }
}
