/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.ros.node.ConnectedNode;

/**
 *
 * @author robot
 */
public class Test extends MainNodeAbstract {

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        try {
            while (true) {
                System.out.println("Turning -90 degrees");
                driver.turn(Math.toRadians(-180), false, true);
                Thread.sleep(1000);
                System.out.println("Turning 90 degrees");
                driver.turn(Math.toRadians(180), false, true);
                System.out.println("finished.");
                Thread.sleep(1000);
            }
        } catch (InterruptedException ex) {
            Logger.getLogger(Test.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    //@Override
    public void onNewCameraRectanglePoints(List<Point> points) {
    }

    @Override
    public void onNewEstimatedPose(Pose estimatedPose) {
    }

    @Override
    public void onNewCameraRectanglePoints(float[] data) {
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
