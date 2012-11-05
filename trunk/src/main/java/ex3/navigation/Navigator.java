package ex3.navigation;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;


public class Navigator extends AbstractNodeMain {

    MessageFactory factory;

    Subscriber<PoseWithCovarianceStamped> estimatedPose;
    Publisher<Twist> rotate;

    @Override
    public void onStart(ConnectedNode connectedNode) {
        factory = connectedNode.getTopicMessageFactory();

        rotate = connectedNode.newPublisher("cmd_vel", Twist._TYPE);

        estimatedPose = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
        estimatedPose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {
                Pose current = t.getPose().getPose();
                Point next = factory.newFromType(Point._TYPE);
                next.setX(15);
                next.setY(15);
                AbstractLocaliser.setMessageFactory(factory);

                Quaternion rot = current.getOrientation();
                System.out.println("curheading: "+AbstractLocaliser.getHeading(rot));
                double bearing = Math.atan2(next.getY() - current.getPosition().getY(), next.getX() - current.getPosition().getX());
                System.out.println("required heading to point: " + bearing);
                if (Math.abs(AbstractLocaliser.getHeading(rot) - bearing) > 0.2){
                    double req;
                    if (bearing < 0) {
                        req = 1.5;
                    } else {
                        req = -1.5;
                    }
                    Twist pub = rotate.newMessage();
                    pub.getAngular().setX(req);
                    rotate.publish(pub);
                    
                }
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Navigator");
    }

}
