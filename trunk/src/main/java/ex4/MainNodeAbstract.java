/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import ex3.PRM;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import java.awt.Dimension;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Int32;
/**
 *
 * @author robot
 */
public abstract class MainNodeAbstract extends AbstractNodeMain {

    public static ConnectedNode node;
    public static MessageFactory messageFactory;
    public static Dimension CAMERA_DIMENSIONS = new Dimension(640, 480);
    public Publisher<Twist> twist_pub;
    public Publisher<PoseStamped> goal;
    protected Driver driver;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ex4/MainNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        //set the connected node
        node = connectedNode;

        //set up the message factory
        messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
        StaticMethods.messageFactory = messageFactory;

        //set publisher for moving the robot
        twist_pub = node.newPublisher("cmd_vel", Twist._TYPE);

        // Publisher for the goal which prm shall use
        goal = node.newPublisher("goal", PoseStamped._TYPE);

        //instantiate the driver with the twist publisher
        driver = new Driver(twist_pub);

        Subscriber<Odometry> odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
        odom.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry t) {
                driver.onNewOdomMessage(t);
            }
        });

        Subscriber<std_msgs.Int32> prmInfo = connectedNode.newSubscriber("goalInfo", std_msgs.Int32._TYPE);
        prmInfo.addMessageListener(new MessageListener<Int32>() {

            @Override
            public void onNewMessage(Int32 t) {
                if (t.getData() == PRM.GOAL_REACHED) {
                    System.out.println("Goal reached");
                } else if (t.getData() == PRM.NO_PATH) {
                    System.out.println("Could not find path");
                } else if (t.getData() == PRM.PATH_FOUND) {
                    System.out.println("Path found.");
                }
            }
        });

        //set up subscriber for the rectangles from the
        Subscriber<std_msgs.Float32MultiArray> cameraRectSubscriber =
                node.newSubscriber("face_rects", std_msgs.Float32MultiArray._TYPE);

        //set the subscriber for the estimated pose
        Subscriber<PoseStamped> estimatedPose = node.newSubscriber("estimated_pose", PoseStamped._TYPE);
        estimatedPose.addMessageListener(new MessageListener<PoseStamped>() {

            @Override
            public void onNewMessage(PoseStamped message) {
                onNewEstimatedPose(StaticMethods.copyPose(message.getPose()));
            }
        });
    }

    public abstract void onNewCameraRectanglePoints(float[] data);

    public abstract void onNewEstimatedPose(Pose estimatedPose);
}
