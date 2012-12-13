package util.networkclient;

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
        moveAlongX(0.5);

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
                Thread.sleep(250);
            }
        } catch (IOException e) {
            System.out.println("IOException. Oops");
            e.printStackTrace();
        } catch (InterruptedException e) {
            System.out.println("InterruptedException. D'oh.");
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
            double speed = Double.parseDouble(tokens[1]);

            System.out.println("Moving along x: " + speed);

            moveAlongX(speed);
        } else if (line.startsWith("RotateZ")) {
            String[] tokens = line.split(":");
            double angle = Double.parseDouble(tokens[1]);

            System.out.println("Rotating along z: " + angle);

            rotate(angle);
        }
    }

    public boolean moveAlongX(double speed) {
        Twist twist = pub.newMessage();
        twist.getLinear().setX(speed);
        pub.publish(twist);
        return true;
    }

    /** Turns clockwise in the given number of radians.
     * Positive goes clockwise. */
    public void rotate(double theta) {
        Twist twist = pub.newMessage();
        twist.getAngular().setZ(-theta);
        pub.publish(twist);
    }
}
