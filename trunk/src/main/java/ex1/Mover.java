package ex1;

import geometry_msgs.Twist;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


public class Mover extends AbstractNodeMain {
    private Publisher<Twist> pub;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("mover");
    }

    @Override
    public void onStart(ConnectedNode node) {
        pub = node.newPublisher("cmd_vel", Twist._TYPE);
        Twist twist = pub.newMessage();
        twist.getLinear().setX(0.5);
        while (true) {
            pub.publish(twist);
            try {
                Thread.sleep(500);
            } catch (InterruptedException ex) {
                Logger.getLogger(Mover.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }




}
