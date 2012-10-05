package util;

import geometry_msgs.Twist;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


public class NetworkJoystickServer extends AbstractNodeMain {

    Publisher<Twist> pub;

    @Override
    public void onStart(ConnectedNode node) {
        pub = node.newPublisher("cmd_vel", Twist._TYPE);
        
        try {
            ServerSocket serv = new ServerSocket(12585);
            System.out.println("Accepting connection...");
            Socket socket = serv.accept();
            System.out.println("Accepted!");
            //BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            Scanner in = new Scanner(socket.getInputStream());
            String line = "";
//            while ((line = in.readLine()) != null) {
            while (true) {
                if (in.hasNext()) {
                    line = in.nextLine();
                    System.out.println("LineRead: " + line);
                    handleInput(line);
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e){}
            }
        } catch (IOException e) {
            System.out.println("IOException. Oops");
            e.printStackTrace();
        }
        System.out.println("Exited main.");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("NetworkJoystick");
    }

    public void handleInput(String line) {
        if (line.startsWith("MoveX")) {
            String[] tokens = line.split(":");
            float speed = Float.parseFloat(tokens[1]);

            System.out.println("\tMoving along x: " + speed);

            moveAlongX(speed);
        } else if (line.startsWith("RotateZ")) {
            String[] tokens = line.split(":");
            float angle = Float.parseFloat(tokens[1]);

            System.out.println("\tRotating along z: " + angle);

            rotate(angle);
        }
    }

    public void moveAlongX(float speed) {
        Twist twist = pub.newMessage();
        twist.getLinear().setX(speed);
        pub.publish(twist);
    }

    /** Turns clockwise in the given number of radians.
     * Positive goes clockwise. */
    public void rotate(double theta) {
        Twist twist = pub.newMessage();
        twist.getAngular().setZ(-theta);
        pub.publish(twist);
    }
}
