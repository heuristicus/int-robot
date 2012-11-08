package ex3.navigation;

import ex3.PRM;
import ex3.PRMUtil;
import ex3.Vertex;
import ex3.pid.PID;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;

import java.util.ArrayList;

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

	public static final double MAX_ROTATION_SPEED = 1.0;
	public static final double MAX_MOVE_SPEED = 1.0;
	public static final double MIN_MOVE_SPEED = 0.1;
	public static final double POINT_REACHED_THRESHOLD = 0.5;
	public static final double POINT_PROXIMITY_THRESHOLD = 2.0; 
	
    MessageFactory factory;
    Pose currentGoal;
    Pose lastEstimate;
        
    Subscriber<Odometry> odom;
    Subscriber<PoseWithCovarianceStamped> estimatedPose;
    Subscriber<PoseStamped> goal;
    Publisher<Twist> rotate;

    PRM prm;
    PID pid;
    ArrayList<Vertex> route;
        
    public Navigator(PRM prm)
    {
    	this.prm = prm;
    }
    
    public Navigator(PRM prm, PID pid){
    	this.prm = prm;
    	this.pid = pid;
    	pid.setOutputLimits(0, MAX_ROTATION_SPEED);
    }
    
    @Override
    public void onStart(ConnectedNode connectedNode) {
    	factory = connectedNode.getTopicMessageFactory();
    	AbstractLocaliser.setFactory(factory);
    	
    	rotate = connectedNode.newPublisher("cmd_vel", Twist._TYPE);
    	goal = connectedNode.newSubscriber("goal", PoseStamped._TYPE);
    	odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
    	estimatedPose = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
    	
    	odom.addMessageListener(new MessageListener<Odometry>() {
			@Override
			public void onNewMessage(Odometry t) {
				// Each time we receive an odometry message, publish a movement to cmd_vel.
				// This is probably not how we should do things - odometry is published even
				// when the robot is not moving which could cause all sorts of weird problems.
				rotate.publish(computeMovementValues());	
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
    	
    	goal.addMessageListener(new MessageListener<PoseStamped>() {
			@Override
			public void onNewMessage(PoseStamped t) {
				// Whenever we receive a new goal, set the PRM goal.
				currentGoal = t.getPose();
				prm.setGoalPosition(currentGoal);
				// Set the prm start position to be as close to our actual position as possible.
				prm.setCurrentPosition(lastEstimate);
			}
		});
    }

    /*
     * Gets a twist message that should be sent to the robot in order to proceed to
     * the next point.
     */
    public Twist computeMovementValues(){
    	Twist pub = rotate.newMessage();
		
    	pub.getAngular().setZ(computeAngularMovement());
    	pub.getLinear().setX(computeLinearMovement());
		
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
    	Twist pub = rotate.newMessage();
    	pub.getAngular().setZ(pid.getOutput());
    	pub.getLinear().setX(computeLinearMovement());
    	return pub;
    }
    
    /*
     * Computes the value of linear movement that should be sent to the robot in a naive way.
     */
    public double computeLinearMovement(){
    	double straightDistanceToWayPoint = PRMUtil.getEuclideanDistance(lastEstimate.getPosition(), currentGoal.getPosition());
    	if (straightDistanceToWayPoint <= POINT_REACHED_THRESHOLD){
    		// If we have reached the point, stop.
    		return 0;
    	}	else if (straightDistanceToWayPoint <= POINT_PROXIMITY_THRESHOLD) {
    		// If we are in the proximity of the waypoint, adjust our speed relative to our distance
    		// from it. Since we don't really care about precision, round the distance value.
    		return boundedSpeed(Math.round(straightDistanceToWayPoint));
    	}	else {
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
    public double computeAngularMovement(){
    	Quaternion rot = lastEstimate.getOrientation();
    	System.out.println("curheading: "+AbstractLocaliser.getHeading(rot));
		double bearing = bearingFromZero(lastEstimate.getPosition(), currentGoal.getPosition());
		System.out.println("required heading to point: " + bearing);
		double req;
		// If we're not aligned particularly well with the heading to the waypoint
		// rotate a bit.
		if (Math.abs(AbstractLocaliser.getHeading(rot) - bearing) > 0.2){
			if (bearing < 0) {
				req = 0.5;
			} else {
				req = -0.5;
			}
		} else { // Otherwise, don't turn.
			req = 0;
		}
		return req;
    }
    
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Navigator");
    }

}
