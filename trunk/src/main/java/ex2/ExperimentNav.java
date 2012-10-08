package ex2;

import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;

public class ExperimentNav extends AbstractNodeMain{

    Subscriber<Odometry> odom;
    Publisher<Twist> move;

    @Override
    public void onStart(ConnectedNode connectedNode) {

        move = connectedNode.newPublisher("cmd_vel", Twist._TYPE);
        odom = connectedNode.newSubscriber("odom", Odometry._TYPE);

        odom.addMessageListener(new MessageListener<Odometry>() {

            int moveflag = 0;

            @Override
            public void onNewMessage(Odometry t) {
                double curX = t.getPose().getPose().getPosition().getX();
                double curY = t.getPose().getPose().getPosition().getY();
                double curHeading = AbstractLocaliser.getHeading(t.getPose().getPose().getOrientation());

                System.out.printf("x: " + curX + "\ty: " + curY + "\theading: " + curHeading + "\n");
                System.out.println("moveflag: " + moveflag);
                Twist fwd = move.newMessage();
                fwd.getLinear().setX(0.5);

                Twist ccwRot = move.newMessage();
                ccwRot.getAngular().setZ(0.5);
                Twist cwRot = move.newMessage();
                cwRot.getAngular().setZ(-0.5);
                
                if(curX < 6.10 && curY < 0.01 && moveflag == 0) {
                    System.out.println("flag 0 fwd");
                    move.publish(fwd);
                }

                if (curX < 6.26 && curX > 6.24 && curY < 0.01 && moveflag == 0){
                    moveflag = 1;
                }

                if (curHeading > -1.00 && moveflag == 1) {
                    move.publish(cwRot);
                }

                if (curHeading < -1.10 && curHeading > -1.20 && moveflag == 1){
                    moveflag = 2;
                }

                if (curX < 8.15 && moveflag == 2){
                    System.out.println("flag 2 fwd");
                    move.publish(fwd);
                }

                if (curX < 8.22 && curX > 8.20 && curY < -4.37 && curY > -4.39 && moveflag == 2){

                    moveflag = 3;
                }

                if (curHeading > -2.45 && moveflag == 3){
                    move.publish(cwRot);
                }

                if (curHeading > -2.60 && curHeading < -2.50 && moveflag == 3){
                    moveflag = 4;
                }

                if (curX > 5.81 && moveflag == 4){
                    System.out.println("flag 4 fwd");
                    move.publish(fwd);
                }

                if (curX > 5.65 && curX < 5.75 && moveflag == 4){
                    moveflag = 5;
                }

                if (curHeading < -1.80 && moveflag == 5){
                    move.publish(ccwRot);
                }

                if (curHeading < -1.60 && curHeading > -1.70 && moveflag == 5){
                    moveflag = 6;
                }

                if (curY > -9.25 && moveflag == 6){
                    move.publish(fwd);
                }

                if (curY > -9.45 && curY < -9.35 && moveflag == 6){
                    moveflag = 7;
                }

                if (curHeading < -0.60 && moveflag == 7){
                    move.publish(ccwRot);
                }

                if (curHeading > -0.55 && curHeading < -0.45 && moveflag == 7){
                    moveflag = 8;
                }

                if (curX < 8.50 && moveflag == 8) {
                    move.publish(fwd);
                }

            }
        });
    }


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("experimentnav");
    }



}
