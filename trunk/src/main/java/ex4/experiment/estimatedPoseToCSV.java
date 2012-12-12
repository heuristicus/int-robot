/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package ex4.experiment;

import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 *
 * @author robot
 */
public class estimatedPoseToCSV  extends AbstractNodeMain {
  BufferedWriter out;
    @Override
    public GraphName getDefaultNodeName() {
            return GraphName.of("estimatedPoseToCSV");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);

        try {
          out = new BufferedWriter(new FileWriter("/data/private/robot/workspace/robotics/logs/estimatedPose.csv", true));
          out.write("--start--");
          out.newLine();
          out.flush();
        } catch (IOException ex) {
            Logger.getLogger(estimatedPoseToCSV.class.getName()).log(Level.SEVERE, null, ex);
        }

        Subscriber<PoseWithCovarianceStamped> estimatedPoseSub;
        estimatedPoseSub = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
        estimatedPoseSub.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {
                try {
                    out.write(t.getHeader().getStamp().secs + "." + t.getHeader().getStamp().nsecs
                            + ", "
                            + t.getPose().getPose().getPosition().getX() + ", "
                            + t.getPose().getPose().getPosition().getY());
                    out.newLine();
                    out.flush();
                } catch (IOException ex) {
                    Logger.getLogger(estimatedPoseToCSV.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        });
    }
}
