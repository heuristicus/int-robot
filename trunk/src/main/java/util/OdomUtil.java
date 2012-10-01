package util;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseWithCovariance;
import nav_msgs.Odometry;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class OdomUtil extends AbstractNodeMain{



    @Override
    public void onStart(ConnectedNode node) {
        Subscriber<Odometry> odom = node.newSubscriber("odom", Odometry._TYPE);

        odom.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry t) {
                PoseWithCovariance posecv = t.getPose();
                Pose pose = posecv.getPose();
                Point pos = pose.getPosition();
                System.out.println("Currently at " + pos.getX() + ", " + pos.getY());
            }

        });

    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("OdomUtil");
    }


}
