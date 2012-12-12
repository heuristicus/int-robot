package ex2;

import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.logging.Level;
import logging.Logger;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;
import std_msgs.Bool;

public class ExperimentNav extends AbstractNodeMain {

    public static final int NSECS_DIVIDER = 100000000;
    Subscriber<Odometry> odom;
    Subscriber<Odometry> ground_truth;
    Subscriber<PoseWithCovarianceStamped> amclexpectedPose;
    Subscriber<PoseStamped> mclexpectedPose;
    Subscriber<std_msgs.Bool> messages;
    Publisher<PoseWithCovarianceStamped> initial_pose_pub;
    Publisher<Twist> move;
    Logger logger;
    Logger checkpointLogger;
    ConnectedNode node;
    boolean newAmclData = false;
    boolean realWorldMode;
    private Odometry[] odomArray = new Odometry[10];
    private PoseStamped mclData;
    private Odometry lastMatchingOdom;
    private Odometry latestOdom;
    private MessageFactory messageFactory;
    private BufferedWriter out;
    private long lastWrite = 0;
    private double writeDelay = 1000;

    public ExperimentNav(boolean realWorld) {
        realWorldMode = realWorld;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.node = connectedNode;

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
        messageFactory = nodeConfiguration.getTopicMessageFactory();

        try {
            System.out.println("Initialising logger...");
            logger = new Logger();
            logger.setAutoFlushing(false);
            logger.logLine(getExperimentDescription());
            checkpointLogger = new Logger("CheckpointLog-", true);
            checkpointLogger.setAutoFlushing(true);
            checkpointLogger.logLine(getExperimentDescription());
        } catch (Exception ex) {
            System.out.println("Logger couldn't initialise. Error: " + ex.toString());
            System.exit(1);
        }


        try {
            out = new BufferedWriter(new FileWriter("/data/private/robot/workspace/robotics/logs/mcl_amcl_gtruth_distance.csv", true));
            out.write("--start--");
            out.newLine();
            out.flush();
        } catch (IOException ex) {
        }

        initial_pose_pub = node.newPublisher("initialpose", PoseWithCovarianceStamped._TYPE);
        move = connectedNode.newPublisher("cmd_vel", Twist._TYPE);
        odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
        ground_truth = connectedNode.newSubscriber("base_pose_ground_truth", Odometry._TYPE);

        publishInitialPose();

        if (realWorldMode) {
            messages = connectedNode.newSubscriber("message", std_msgs.Bool._TYPE);

            messages.addMessageListener(new MessageListener<Bool>() {

                @Override
                public void onNewMessage(Bool t) {
                    System.out.println("Received checkpoint message!");
                    if (mclData != null) {
                        checkpointReached(latestOdom);
                    }
                }
            });
        }

        amclexpectedPose = connectedNode.newSubscriber("/amcl_pose", PoseWithCovarianceStamped._TYPE);
        amclexpectedPose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {

            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {
                PoseStamped poseStamped = messageFactory.newFromType(PoseStamped._TYPE);
                poseStamped.setHeader(t.getHeader());
                poseStamped.setPose(t.getPose().getPose());
                mclData = poseStamped;
                newAmclData = true;
            }
        });


           mclexpectedPose = connectedNode.newSubscriber("/estimated_pose", PoseStamped._TYPE);
        mclexpectedPose.addMessageListener(new MessageListener<PoseStamped>() {

            @Override
            public void onNewMessage(PoseStamped t) {
                mclData = t;
                newAmclData = true;
            }
        });

        odom.addMessageListener(new MessageListener<Odometry>() {

            int moveflag = 0;

            @Override
            public void onNewMessage(Odometry odom) {
                if (realWorldMode) {
                    realWorldLogging(odom);
                } else {
                    double curX = odom.getPose().getPose().getPosition().getX();
                    double curY = odom.getPose().getPose().getPosition().getY();
                    double curHeading = AbstractLocaliser.getHeading(odom.getPose().getPose().getOrientation());

                    moveflag = experimentPoints(curX, curY, curHeading, moveflag);
                }
            }
        });

        ground_truth.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry odom) {
                stageLogging(odom);
            }
        });
    }

    public void realWorldLogging(Odometry odom_ground_truth) {
        latestOdom = odom_ground_truth;
        if (mclData != null && newAmclData) {
            logger.logLine(getTimeStampWithPoseString(odom_ground_truth) + "\t\t" + getTimeStampWithPoseString(mclData));
            newAmclData = false;
        }
    }

    public void stageLogging(Odometry odom) {
        double y_temp = odom.getPose().getPose().getPosition().getY() * -1.0;
        odom.getPose().getPose().getPosition().setY(odom.getPose().getPose().getPosition().getX());
        odom.getPose().getPose().getPosition().setX(y_temp);

        odomArray[odom.getHeader().getStamp().nsecs / NSECS_DIVIDER] = odom;
        if (mclData != null
                && odomArray[mclData.getHeader().getStamp().nsecs / NSECS_DIVIDER] != null
                && newAmclData) {
            int amclTime = mclData.getHeader().getStamp().nsecs / NSECS_DIVIDER;

            if(System.currentTimeMillis() - lastWrite < writeDelay){
                return;
            }
            
            //System.out.println(getTimeStampWithPoseString(amclData) + "\t\t" + getTimeStampWithPoseString(odomArray[amclTime]));
            String timeDistanceString = getTimeStamptWithDistance(mclData, odomArray[amclTime]);
            System.out.println(timeDistanceString);

            try {
                lastWrite = System.currentTimeMillis();
                out.write(timeDistanceString);
                out.newLine();
                out.flush();
            } catch (IOException ex) {
                java.util.logging.Logger.getLogger(ExperimentNav.class.getName()).log(Level.SEVERE, null, ex);
            }

            if (LocalisationUtil.timeStampEqual(odomArray[amclTime].getHeader().getStamp(), mclData.getHeader().getStamp())) {
                logger.logLine(getTimeStampWithPoseString(mclData) + "\t"
                        + getTimeStampWithPoseString(odomArray[amclTime]));
                lastMatchingOdom = odomArray[amclTime];
            } else {
                System.out.println("ERROR! Something bad is happening inside onNewMessage()");
                System.err.println("ERROR! Something bad is happening inside onNewMessage()");
                logger.logLine("ERROR! Encountered inequality in timestamps: ");
                logger.logLine("AMCL: " + getTimeStampWithPoseString(mclData) + "\t\t"
                        + "ODOM: " + getTimeStampWithPoseString(odomArray[amclTime]));
                checkpointLogger.logLine("ERROR! Encountered inequality in timestamps: ");
                checkpointLogger.logLine("AMCL: " + getTimeStampWithPoseString(mclData) + "\t\t"
                        + "ODOM: " + getTimeStampWithPoseString(odomArray[amclTime]));
            }
            newAmclData = false;
        }

    }

    public int experimentPoints(double curX, double curY, double curHeading, int flag) {
        int moveflag = flag;
        Twist fwd = move.newMessage();
        fwd.getLinear().setX(0.5);

        Twist ccwRot = move.newMessage();
        ccwRot.getAngular().setZ(0.5);
        Twist cwRot = move.newMessage();
        cwRot.getAngular().setZ(-0.5);

        if (curX < 6.10 && curY < 0.01 && moveflag == 0) {
            move.publish(fwd);
        }

        if (curX < 6.26 && curX > 6.24 && curY < 0.01 && moveflag == 0) {
            moveflag = 1;
            checkpointReached();
        }

        if (curHeading > -1.00 && moveflag == 1) {
            move.publish(cwRot);
        }

        if (curHeading < -1.10 && curHeading > -1.20 && moveflag == 1) {
            moveflag = 2;
            checkpointReached();
        }

        if (curX < 8.15 && moveflag == 2) {
            move.publish(fwd);
        }

        if (curX < 8.22 && curX > 8.20 && curY < -4.37 && curY > -4.39 && moveflag == 2) {
            moveflag = 3;
            checkpointReached();
        }

        if (curHeading > -2.45 && moveflag == 3) {
            move.publish(cwRot);
        }

        if (curHeading > -2.60 && curHeading < -2.50 && moveflag == 3) {
            moveflag = 4;
            checkpointReached();
        }

        if (curX > 5.81 && moveflag == 4) {
            move.publish(fwd);
        }

        if (curX > 5.65 && curX < 5.75 && moveflag == 4) {
            moveflag = 5;
            checkpointReached();
        }

        if (curHeading < -1.80 && moveflag == 5) {
            move.publish(ccwRot);
        }

        if (curHeading < -1.60 && curHeading > -1.70 && moveflag == 5) {
            moveflag = 6;
            checkpointReached();
        }

        if (curY > -9.25 && moveflag == 6) {
            move.publish(fwd);
        }

        if (curY > -9.45 && curY < -9.35 && moveflag == 6) {
            moveflag = 7;
            checkpointReached();
        }

        if (curHeading < -0.60 && moveflag == 7) {
            move.publish(ccwRot);
        }

        if (curHeading > -0.55 && curHeading < -0.45 && moveflag == 7) {
            moveflag = 8;
            checkpointReached();
        }

        if (curX < 8.50 && moveflag == 8) {
            move.publish(fwd);
        }

        if (curX > 8.55 && curX < 8.65 && curY < -11.05 && curY > -11.15 && moveflag == 8) {
            checkpointReached();
            try {
                out.close();
            } catch (IOException ex) {
                java.util.logging.Logger.getLogger(ExperimentNav.class.getName()).log(Level.SEVERE, null, ex);
            }
            node.shutdown();
        }

        return moveflag;
    }

    public String getExperimentDescription() {
        final String NL = "\n";

        String desc = "";
        desc += "PFLocaliser Rotation Noise: " + PFLocaliser.ROTATION_NOISE + NL;
        desc += "PFLocaliser Position Noise: " + PFLocaliser.POSITION_NOISE + NL;
        desc += "PFLocaliser NumOfParticles: " + PFLocaliser.PARTICLE_NUMBER + NL;
        desc += "PFLocaliser ClusterThreshold: " + PFLocaliser.CLUSTER_THRESHOLD + NL;
        desc += "AbstractLocaliser RotationNoise: " + AbstractLocaliser.getRotationNoise() + NL;
        desc += "AbstractLocaliser PositionNoise: " + AbstractLocaliser.getPositionNoise();
        desc += "\n\n";
        return desc;
    }

    public void checkpointReached(Odometry odom) {
        checkpointLogger.logLine(getTimeStampWithPoseString(mclData)
                + "\t" + getTimeStampWithPoseString(odom));
    }

    public void checkpointReached() {
        checkpointLogger.logLine(getTimeStampWithPoseString(mclData)
                + "\t" + getTimeStampWithPoseString(lastMatchingOdom));
    }

    public String getTimeStampWithPoseString(PoseStamped pose) {
        return LocalisationUtil.getTimeStampWithPose(
                pose.getPose(),
                pose.getHeader().getStamp());
    }

    public String getTimeStamptWithDistance(PoseStamped amcl, Odometry ground_Truth) {
        return amcl.getHeader().getStamp().secs + "."
                + amcl.getHeader().getStamp().nsecs + ", " + " distance:"
                + distanceBetweenPoints(amcl.getPose().getPosition(),
                ground_Truth.getPose().getPose().getPosition());
    }

    public double distanceBetweenPoints(Point p1, Point p2) {
        double x = p1.getX() - p2.getX();
        double y = p1.getY() - p2.getY();
        return Math.sqrt((x * x) + (y * y));
    }

    public String getTimeStampWithPoseString(Odometry pose) {
        return LocalisationUtil.getTimeStampWithPose(
                pose.getPose(),
                pose.getHeader().getStamp());
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("experimentnav");
    }

    @Override
    public void onShutdown(Node node) {
        logger.close();
    }

    public Quaternion createQuaternion() {
        Quaternion q = messageFactory.newFromType(Quaternion._TYPE);

        // Set up 'identity' (blank) quaternion
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        return q;
    }

    public Quaternion rotateQuaternion(Quaternion q_orig, double yaw) {
        // Create a temporary Quaternion to represent the change in heading
        Quaternion q_headingChange = createQuaternion();

        double p = 0;
        double y = yaw / 2.0;
        double r = 0;

        double sinp = Math.sin(p);
        double siny = Math.sin(y);
        double sinr = Math.sin(r);
        double cosp = Math.cos(p);
        double cosy = Math.cos(y);
        double cosr = Math.cos(r);

        q_headingChange.setX(sinr * cosp * cosy - cosr * sinp * siny);
        q_headingChange.setY(cosr * sinp * cosy + sinr * cosp * siny);
        q_headingChange.setZ(cosr * cosp * siny - sinr * sinp * cosy);
        q_headingChange.setW(cosr * cosp * cosy + sinr * sinp * siny);

        // Multiply new (heading-only) quaternion by the existing (pitch and bank) quaternion
        // Order is important! Original orientation is the second argument;
        // rotation which will be applied to the quaternion is the first argument.
        return multiply_quaternions(q_headingChange, q_orig);
    }

    private Quaternion multiply_quaternions(Quaternion qa, Quaternion qb) {
        Quaternion combined = createQuaternion();

        combined.setW(qa.getW() * qb.getW() - qa.getX() * qb.getX() - qa.getY() * qb.getY() - qa.getZ() * qb.getZ());
        combined.setX(qa.getX() * qb.getW() + qa.getW() * qb.getX() + qa.getY() * qb.getZ() - qa.getZ() * qb.getY());
        combined.setY(qa.getW() * qb.getY() - qa.getX() * qb.getZ() + qa.getY() * qb.getW() + qa.getZ() * qb.getX());
        combined.setZ(qa.getW() * qb.getZ() + qa.getX() * qb.getY() - qa.getY() * qb.getX() + qa.getZ() * qb.getW());
        return combined;

    }

    private void publishInitialPose() {
        PoseWithCovarianceStamped initialPose = initial_pose_pub.newMessage();
        initialPose.getPose().getPose().getPosition().setX(12.75);
        initialPose.getPose().getPose().getPosition().setY(4.075);
        initialPose.getPose().getPose().setOrientation(rotateQuaternion(createQuaternion(), Math.toRadians(128.92)));

        double[] cov = new double[36]; // 36 comes from the msg definition, and is fixed
        for (int i = 0; i < cov.length; i++) {
            cov[i] = 0.0;
        }
        initialPose.getPose().setCovariance(cov);

        initialPose.getHeader().setFrameId("/map");

        long timeNow = System.currentTimeMillis();

        while (System.currentTimeMillis() - timeNow < 1000) {
            initial_pose_pub.publish(initialPose);
        }
        System.out.println("Published intitial pose: X: " + initialPose.getPose().getPose().getPosition().getX()
                + ", Y: " + initialPose.getPose().getPose().getPosition().getY());
    }
}
