/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import geometry_msgs.Twist;
import java.util.logging.Level;
import java.util.logging.Logger;
import nav_msgs.Odometry;
import org.ros.node.topic.Publisher;

/**
 *
 * @author robot
 */
public class Driver {

    //the maximum speed when driving
    //the twist publisher
    private Publisher<Twist> twistPublisher;
    private double maxTurnRate = 0.5;
    private Double angleR;
    private Double targetHeading;
    private boolean targetReached = false;

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
            return;
        }

        Twist twist = twistPublisher.newMessage();
        double turnRate = 0;
        double diff = currentHeading - targetHeading;

        if (Math.abs(diff) > Math.toRadians(10)) {
            targetReached = false;
            if (diff > 0) {
                turnRate = -maxTurnRate;
            } else {
                turnRate = maxTurnRate;
            }
        } else {
            angleR = null;
            targetReached = true;
            System.out.println("No longer need to turn!");
        }
        twist.getAngular().setZ(turnRate);
        System.out.println("Turning: " + twist.getAngular().getZ());
        twistPublisher.publish(twist);
    }

    public void turn(double angle, boolean useShortest, boolean wait) {
        targetHeading = null;
        angleR = -angle;
        if (useShortest) {
            if (angleR < -Math.PI) {
                angle += 2 * Math.PI;
            } else if (angle > Math.PI) {
                angleR -= 2 * Math.PI;
            }
        }
        targetReached = false;

        if (wait) {
            while (!getTargetReached()) {
                System.out.println("Turning...");
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ex) {
                    Logger.getLogger(Driver.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    public boolean getTargetReached() {
        return targetReached;
    }
}
