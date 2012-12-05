/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import geometry_msgs.Twist;
import java.util.logging.Level;
import java.util.logging.Logger;
import launcher.RunParams;
import nav_msgs.Odometry;
import org.ros.node.topic.Publisher;

/**
 *
 * @author robot
 */
public class Driver {

    public static final double DRIVER_HEADING_THRESHOLD = RunParams.getDouble("DRIVER_HEADING_THRESHOLD");
    public static final double DRIVER_MAX_TURN_RATE = RunParams.getDouble("DRIVER_MAX_TURN_RATE");
    //the maximum speed when driving
    //the twist publisher
    private Publisher<Twist> twistPublisher;
    private Double angleR;
    private Double targetHeading;
    private boolean targetReached = false;
    private double angleTurned = 0;

    /**
     * Constructor
     * @param twistPublisher The publisher to publish twists to
     */
    public Driver(Publisher<Twist> twistPublisher) {
        this.twistPublisher = twistPublisher;
    }

    public void onNewOdomMessage(Odometry t) {
        if (angleR == null) {
            return;
        }

        double currentHeading = StaticMethods.getHeading(t.getPose().getPose().getOrientation());
        if (targetHeading == null) {
            targetHeading = currentHeading - angleR;
        }

        angleTurned = (targetHeading + angleR) - currentHeading;

        Twist twist = twistPublisher.newMessage();
        double turnRate = 0;
        double diff = currentHeading - targetHeading;

        Printer.println("Diff in newOdom is: " + diff, "CYANF");
        if (Math.abs(diff) > DRIVER_HEADING_THRESHOLD) {
            targetReached = false;
            if (diff > 0) {
                turnRate = -DRIVER_MAX_TURN_RATE; // Clockwise
            } else {
                turnRate = DRIVER_MAX_TURN_RATE; // Counter-clockwise
            }
        } else {
            angleR = null;
            targetReached = true;
            System.out.println("No longer need to turn!");
        }
        twist.getAngular().setZ(turnRate);
        System.out.println("Turning " + (diff > 0 ? "right" : "left") + ": " + twist.getAngular().getZ());
        twistPublisher.publish(twist);
    }

    public void turn(double angle, boolean useShortest, boolean wait) {
        targetHeading = null;
        angleTurned = 0;
        angleR = -angle;
        if (useShortest) {
            if (angleR < -Math.PI) {
                angleR += 2 * Math.PI;
            } else if (angle > Math.PI) {
                angleR -= 2 * Math.PI;
            }
        }
        targetReached = false;

        if (wait) {
            while (!isTargetReached()) {
                System.out.println("Turning...");
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ex) {
                    Logger.getLogger(Driver.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    public double getAngleTurned() {
        return angleTurned;
    }

    public void stopTurning() {
        angleR = null;
        targetReached = true;
    }

    public boolean isTargetReached() {
        return targetReached;
    }
}
