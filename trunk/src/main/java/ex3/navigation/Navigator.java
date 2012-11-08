package ex3.navigation;

import ex3.PRM;
import ex3.PRMUtil;
import ex3.pid.PID;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;


import nav_msgs.Odometry;

import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import pf.AbstractLocaliser;

public class Navigator extends AbstractNodeMain {

    public static final double MAX_ROTATION_SPEED = 1.5;
    public static final double MAX_MOVE_SPEED = 1.0;
    public static final double MIN_MOVE_SPEED = 0.1;
    public static final double POINT_REACHED_THRESHOLD = 0.5;
    public static final double POINT_PROXIMITY_THRESHOLD = 2.0;


    MessageFactory factory;
    Pose wayPoint;
    Pose lastEstimate;
    Pose goalPoint;
    boolean active = false;
    boolean turnOnSpot = false;
    double distanceToWaypoint;
    double rotationToWaypoint;
    PRM prm;
    PID pid;
    PoseArray route;

    Subscriber<Odometry> odom;
    Subscriber<PoseWithCovarianceStamped> estimatedPose;
    Subscriber<PoseArray> routeSub;
    Publisher<Twist> movement;

    public Navigator(PRM prm) {
        this.prm = prm;
    }

//    public Navigator(PRM prm, PID pid) {
//        this.prm = prm;
//        this.pid = pid;
//        pid.setOutputLimits(0, MAX_ROTATION_SPEED);
//    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        factory = connectedNode.getTopicMessageFactory();
        AbstractLocaliser.setFactory(factory);

        movement = connectedNode.newPublisher("cmd_vel", Twist._TYPE);
        odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
        estimatedPose = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
        routeSub = connectedNode.newSubscriber("route", PoseArray._TYPE);

        odom.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry t) {
                // Each time we receive an odometry message, publish a movement to cmd_vel.
                // This is probably not how we should do things - odometry is published even
                // when the robot is not moving which could cause all sorts of weird problems.
                if (active && route != null) {
                    distanceToWaypoint = PRMUtil.getEuclideanDistance(lastEstimate.getPosition(), wayPoint.getPosition());
                    if (distanceToWaypoint <= POINT_REACHED_THRESHOLD){
                        if (nextWayPoint() == false) { // we have reached the goal.
                            System.out.println("Goal reached.");
                            active = false;
                            route = null;
                        } else { // Not yet reached the end of the path
                            turnOnSpot = true;
                            System.out.println("Proceeding to next waypoint.");
                        }
                    }
                    movement.publish(computeMovementValues());
                }
            }
        });

        routeSub.addMessageListener(new MessageListener<PoseArray>() {
            @Override
            public void onNewMessage(PoseArray t) {
                route = t;
                initRoute();
            }
        });

        estimatedPose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {
                // Each time we get an update for the pose estimate, we update our position
                lastEstimate = t.getPose().getPose();
                prm.setCurrentPosition(lastEstimate);
            }
        });

    }

    /*
     * Initialises the navigator to follow the route from the current location to
     * the end point.
     */
    public void initRoute(){
        nextWayPoint();
        goalPoint = route.getPoses().get(route.getPoses().size() - 1);
        active = true;
    }

    /*
     * Gets the next waypoint on the route by removing the head of the list. Returns
     * true if there is still a waypoint left, setting the waypoint variable to
     * the node at the head of the list. Returns false if the list is empty,
     * implying that we have reached the goal.
     */
    public boolean nextWayPoint(){
        if (route.getPoses().isEmpty()){
            return false;
        } else {
            wayPoint = route.getPoses().remove(0);
            return true;
        }
    }

    /*
     * Gets a twist message that should be sent to the robot in order to proceed to
     * the next point.
     */
    public Twist computeMovementValues() {
        Twist pub = movement.newMessage();

        System.out.println("Next point: " + wayPoint.getPosition().getX() + "," + wayPoint.getPosition().getY());

        double rotReq = computeAngularMovement();
        double moveReq = computeLinearMovement();

        System.out.println("Publishing rotation: " + rotReq + " and movement: " + moveReq);

        pub.getAngular().setZ(rotReq);

        System.out.println("Rotating on waypoint: " + turnOnSpot);

        if (turnOnSpot){
            System.out.println("Rotation to next waypoint: " + rotationToWaypoint);
            if (rotationToWaypoint < 0.1){
                turnOnSpot = false;
            }
        } else {
            pub.getLinear().setX(moveReq);
        }
        
        return pub;
    }

    /*
     * Calculates the bearing from zero from one point to another. Zero is 
     * facing directly east.
     */
    public double bearingFromZero(Point p1, Point p2)
    {
    	return Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
    }
    
    /*
     * Uses a PID controller to set the rotation value sent to cmd_vel. 
     * The standard method for linear movement calculation is used.
     */
    public Twist PIDcontrol(){
    	Twist pub = movement.newMessage();

        double rotReq = pid.getOutput();

        System.out.println("PID rotation request = " + rotReq);

    	pub.getAngular().setZ(rotReq);
    	pub.getLinear().setX(computeLinearMovement());
    	return pub;
    }
    
    /*
     * Computes the value of linear movement that should be sent to the robot in a naive way.
     */
    public double computeLinearMovement(){
        if (distanceToWaypoint < POINT_REACHED_THRESHOLD){
            // If we have reached the point, stop.
            return 0;
        } else if (distanceToWaypoint <= POINT_PROXIMITY_THRESHOLD) {
            // If we are in the proximity of the waypoint, adjust our speed relative to our distance
            // from it. Since we don't really care about precision, round the distance value.
            return boundedSpeed(Math.round(distanceToWaypoint));
        } else {
            // If we are not in the proximity of the waypoint, move as fast as we are allowed to.
            return MAX_MOVE_SPEED;
        }
    }
    
    /*
     * Returns a speed value that is within the default bounds.
     */
    public double boundedSpeed(double speedReq){
    	if (speedReq < MIN_MOVE_SPEED){
    		return MIN_MOVE_SPEED;
    	} else if (speedReq > MAX_MOVE_SPEED){
    		return MAX_MOVE_SPEED;
    	} else {
    		return speedReq;
    	}
    }
    
    /*
     * Computes the angular movement required to keep the robot on course for the point
     * that it is heading towards.
     */
    public double computeAngularMovement() {
        Quaternion rot = lastEstimate.getOrientation();
        double robotHeading = AbstractLocaliser.getHeading(rot);
        double bearing = bearingFromZero(lastEstimate.getPosition(), wayPoint.getPosition());
        System.out.println("curheading: " + robotHeading);
        System.out.println("bearing to point: " + bearing);
        double req = 0;
        double diff = bearing - robotHeading;
        // If we're not aligned particularly well with the heading to the waypoint
        // rotate a bit.
        if (Math.toDegrees(diff) > 180) {
            req = diff - Math.toRadians(360);
        } else if (Math.toDegrees(diff) < -180){
            req = diff + Math.toRadians(360);
        } else {
            req = diff;
        }

        rotationToWaypoint = req;

        return req;
    }
    
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Navigator");
    }

}
