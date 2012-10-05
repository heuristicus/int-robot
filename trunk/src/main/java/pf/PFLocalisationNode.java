package pf;

/**
 *
 * @author Mark Rowan
 * This is a ROS Node (extends AbstractNodeMain) which receives LaserScans and Maps
 * and passes them to an AbstractLocaliser for processing to obtain a location estimate.
 * 
 */

import ex2.PFLocaliser;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.message.MessageListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.PoseWithCovarianceStamped; // Pose estimates (with covariance)
import geometry_msgs.PoseArray; // Pose estimates
import geometry_msgs.Quaternion; // Rotation
import geometry_msgs.TransformStamped;
import tf.tfMessage; // Transform from odom to map
import sensor_msgs.LaserScan; // Laser scans
import nav_msgs.OccupancyGrid; // Map

public class PFLocalisationNode extends AbstractNodeMain {

    // Settings
    private static final double PUBLISH_DELTA = 0.1; // // Minimum change (m/radians) before publishing new particle cloud and pose

    // Fields
    private PFLocaliser pf; // PFLocaliser needs to be implemented by you!
    private LaserScan latestScan; // Most recent laser scan (as a field so it's accessible to all methods)
    private PoseWithCovarianceStamped last_published_pose; // Last-published pose estimate
    private TransformStamped last_odom_transform; // Last-published Transform

    // Flags
    private boolean mapReceived = false;
    private boolean initialPoseReceived = false;

    // ROS Node
    protected static ConnectedNode node; // node can be used to access a MessageFactory for creating Messages

    // Publishers
    private Publisher<PoseWithCovarianceStamped> amcl_pose;
    private Publisher<PoseArray> particlecloud;
    private Publisher<tfMessage> transformpublisher;


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("pf/PFLocalisationNode"); // Give this Node a ROS graph name
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Set Node
        PFLocalisationNode.node = connectedNode;

        // Initialise particle filter
        pf = new PFLocaliser();

        // Initialise fields
        latestScan = node.getTopicMessageFactory().newFromType(LaserScan._TYPE);
        last_odom_transform = node.getTopicMessageFactory().newFromType(TransformStamped._TYPE);

        // Create Publishers
        amcl_pose = node.newPublisher("amcl_pose", PoseWithCovarianceStamped._TYPE);
        particlecloud = node.newPublisher("particlecloud", PoseArray._TYPE);
        transformpublisher = node.newPublisher("tf", tfMessage._TYPE);

        // Create Subscribers
        Subscriber<LaserScan> laser = node.newSubscriber("base_scan", LaserScan._TYPE);
        laser.addMessageListener(new MessageListener<LaserScan>() {

            @Override
            public void onNewMessage(LaserScan message) {
                latestScan = message; // Store latest scan for use by Transform subscriber when updating PF

                if (mapReceived && initialPoseReceived && sufficientMovementDetected(pf.getPose())) {
                    // Publish the new pose
                    amcl_pose.publish(pf.getPose());

                    // Update record of previously-published pose
                    last_published_pose = pf.getPose();

                    // Get updated particle cloud and publish it
                    particlecloud.publish(pf.getParticleCloud());

                    // Get updated transform and publish it
                    // System.out.println(" * Publishing transform from odom to map");
                    transformpublisher.publish(pf.getTransform());
                    
                    double x = pf.getPose().getPose().getPose().getPosition().getX();
                    double y = pf.getPose().getPose().getPose().getPosition().getY();
                    double heading = Math.toDegrees(PFLocaliser.getHeading(pf.getPose().getPose().getPose().getOrientation()));
                    System.out.println("\nPublishing pose:\nLoc: " + x + ", " + y + "\nHeading: " + heading);
                }
            }
        });

        Subscriber<PoseWithCovarianceStamped> initialpose = node.newSubscriber("initialpose", PoseWithCovarianceStamped._TYPE);
        initialpose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {

            @Override
            public void onNewMessage(PoseWithCovarianceStamped message) {
                pf.setInitialPose(message);
                initialPoseReceived = true;
                last_published_pose = node.getTopicMessageFactory().newFromType(PoseWithCovarianceStamped._TYPE);
            }
        });

        Subscriber<OccupancyGrid> map = node.newSubscriber("map", OccupancyGrid._TYPE);
        map.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid message) {
                pf.setMap(message);
                mapReceived = true;
            }
        });


        Subscriber<tfMessage> tf = node.newSubscriber("tf", tfMessage._TYPE);
        tf.addMessageListener(new MessageListener<tfMessage>() {

            @Override
            public void onNewMessage(tfMessage message) {
                // Update the odometry and particle cloud whenever there is a new odometry Transform
                if (mapReceived && initialPoseReceived) {

                    // Search backwards through transform messages to find most recent one from odometry
                    for (int index = message.getTransforms().size() - 1; index >= 0; index--) {
                        TransformStamped tf = message.getTransforms().get(index);

                        // Find a transform message from odom to base
                        if (tf.getHeader().getFrameId().equals("/odom") && tf.getChildFrameId().contains("/base")) {

                            // Check whether this is the same TF we last used to update odometry
                            if ( !tf.getHeader().getStamp().equals(last_odom_transform.getHeader().getStamp()) ) {
                                // Update record of last odometry Transform message for future comparison
                                last_odom_transform = tf;

                                // Set the particle filter's 'current' odometry transform
                                pf.setTransform(message);

                                // Add the predicted motion to each of the particles in particlecloud
                                pf.updateOdom(tf);

                                // Update the particle filter to calculate new odom-to-map transform
                                // System.out.println("\n * Updating PF:");
                                pf.updatePF(tf, latestScan);
                            } else {
                                index = 0; // Break the loop
                            }
                        }
                    }
                }
            }
        });

        // Report progress, and block until map received
        System.out.println ("\n *** PFLocalisation node " + node.getName() + " initialised ***");
        System.out.println("Waiting for map. Please run a map_server (rosrun map_server map_server <map>)");
        while (!mapReceived) {
            try {Thread.sleep(1000);} catch (InterruptedException e) {}
        }
    }

    private boolean sufficientMovementDetected(PoseWithCovarianceStamped latest_pose) {
        // Check that minimum required amount of movement has occurred before re-publishing
        double latest_x = latest_pose.getPose().getPose().getPosition().getX();
        double latest_y = latest_pose.getPose().getPose().getPosition().getY();
        double prev_x = last_published_pose.getPose().getPose().getPosition().getX();
        double prev_y = last_published_pose.getPose().getPose().getPosition().getY();        
        double location_delta = Math.abs(latest_x - prev_x) + Math.abs(latest_y - prev_y);

        // Also check for difference in orientation: Take a zero-quaternion,
        // rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        Quaternion latest_rot = latest_pose.getPose().getPose().getOrientation();
        Quaternion prev_rot = last_published_pose.getPose().getPose().getOrientation();

        Quaternion q = AbstractLocaliser.createQuaternion();
        q = AbstractLocaliser.rotateQuaternion(q, AbstractLocaliser.getHeading(latest_rot)); // Rotate forward
        q = AbstractLocaliser.rotateQuaternion(q, -AbstractLocaliser.getHeading(prev_rot)); // Rotate backward
        double heading_delta = Math.abs(AbstractLocaliser.getHeading(q));

        return (location_delta > PUBLISH_DELTA || heading_delta > PUBLISH_DELTA);
    }
}
