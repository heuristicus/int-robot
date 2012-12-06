/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import geometry_msgs.Twist;
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
//    private Double angleR;
    private Double targetHeading;
    private boolean targetReached = false;
    private double angleTurned;
    private double requestAngleMagnitude;
    private Odometry lastOdom = null;
    private boolean active = true;

    public enum TurnDirection { LEFT, RIGHT }
    public TurnDirection direction;

    /**
     * Constructor
     * @param twistPublisher The publisher to publish twists to
     */
    public Driver(Publisher<Twist> twistPublisher) {
        this.twistPublisher = twistPublisher;
    }

    public void onNewOdomMessage(Odometry t) {
        if (targetHeading == null || ! active || targetReached) {
            return;
        }
        
        double currentHeading = StaticMethods.getHeading(t.getPose().getPose().getOrientation());

        if(lastOdom != null){
            double lastOdomHeading = StaticMethods.getHeading(lastOdom.getPose().getPose().getOrientation());
            //Printer.println("currentHeading: " + currentHeading, "REDF");
            //Printer.println("lastOdomHeading: " + lastOdomHeading, "REDF");
            if (currentHeading < 0 && lastOdomHeading > 0) {
                //Printer.println("Special case heading", "REDF");
                if (lastOdomHeading >= Math.PI / 2) {
                  //  Printer.println("2Pi - " + currentHeading + " - " + lastOdomHeading + " = "
                  //          + (Math.PI * 2 - currentHeading - lastOdomHeading), "REDF");
                    angleTurned += Math.PI * 2 + currentHeading - lastOdomHeading;
                } else {
                    angleTurned += lastOdomHeading - currentHeading;
                }
            } else if (lastOdomHeading < 0 && currentHeading > 0) {
                if (currentHeading >= Math.PI / 2) {
                    angleTurned += Math.PI * 2 - currentHeading + lastOdomHeading;
                } else {
                    angleTurned += currentHeading - lastOdomHeading;
                }
            } else {
                //Printer.println("Standard case heading", "REDF");
                angleTurned += Math.abs(currentHeading - lastOdomHeading);
            }
        }

        Twist twist = twistPublisher.newMessage();
        double turnReq = 0;

        //Printer.println("AngleTurned is " + angleTurned + ", magnitude-angleTurned: " + (requestAngleMagnitude - angleTurned), "REDF");

        if (requestAngleMagnitude - angleTurned > DRIVER_HEADING_THRESHOLD){
            targetReached = false;
            if (direction == TurnDirection.LEFT) {
                turnReq = DRIVER_MAX_TURN_RATE; // Clockwise
            } else {
                turnReq = -DRIVER_MAX_TURN_RATE; // Counter-clockwise
            }
        } else {
            targetReached = true;
            lastOdom = null;
            targetHeading = null;
//            System.out.println("No longer need to turn!");
        }
        twist.getAngular().setZ(turnReq);
  //      Printer.println("Turning " + direction.toString(), "REDF");
        twistPublisher.publish(twist);
        lastOdom = t;
    }

     public void turn(double heading, double angle) {
         angleTurned = 0;
         requestAngleMagnitude = Math.abs(angle);
         double targetAngle = heading + angle;
         direction = angle < 0 ? TurnDirection.RIGHT : TurnDirection.LEFT;
//         System.out.println("angle is " + angle + "Turn direction " + direction.toString());
         Printer.println("Driver.turn called with heading: " + heading + " angle: " + angle + " targetAngle: " + targetAngle, "REDF");
         if (targetAngle > Math.PI) {
             targetAngle = -Math.PI + (targetAngle - Math.PI);
         } else if (targetAngle < -Math.PI) {
             targetAngle = Math.PI + (targetAngle + Math.PI);
         }

         Printer.println("Updated targetangle " + targetAngle, "REDF");
         this.targetHeading = targetAngle;
         active = true;
    }

    public double getAngleTurned() {
        return angleTurned;
    }

    /** Resuming after a temporary pause */
    public void resumeTurning() {
        active = true;
    }

    /** A temporary pause in turning */
    public void pauseTurning() {
        active = false;
    }

    /** Cancel this turning action fully (as opposed to pause, which is temporary) */
    public void stopTurning() {
        targetReached = true;
    }

    public boolean isTargetReached() {
        return targetReached;
    }
}
