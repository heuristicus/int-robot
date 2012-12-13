package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.topic.Publisher;
import pf.AbstractLocaliser;
import pf.SensorModel;

public class LocalisationUtil {

    private MessageFactory messageFactory;
    private Random randGen;

    private double positionNoise;
    private double rotationNoise;


    public LocalisationUtil(MessageFactory msgFactory, Random rand,
            double positionNoise, double rotationNoise) {
        this.messageFactory = msgFactory;
        this.randGen = rand;
        this.positionNoise = positionNoise;
        this.rotationNoise = rotationNoise;
    }

    public static double getGaussian(double mean, double stdDev, Random rGen){
        double rand = rGen.nextGaussian();
        rand *= stdDev;
        rand += mean;

        return rand;
    }

    /** Calculates a random value under a distribution specified by the given
     * mean and standard deviation */
    public double getGaussian(double mean, double stdDev) {
        return getGaussian(mean, stdDev, randGen);
    }

    /** Applies noise to all particles and returns a new noisier list */
    public List<Pose> applyNoise(List<Pose> poses) {
        ArrayList<Pose> newPoses = new ArrayList<Pose>(poses.size());
        for (int i = 0; i < poses.size(); i++) {
            Pose newPose = applyNoise(poses.get(i));
            newPoses.add(newPose);
        }
        return newPoses;
    }

    /** Returns a new pose which is the given pose with noise added. */
    public Pose applyNoise(Pose pose) {
        Pose rPose = messageFactory.newFromType(Pose._TYPE);

        rPose.setOrientation(applyQuaternionNoiseGaussian(pose.getOrientation()));
        rPose.setPosition(applyPointNoiseGaussian(pose.getPosition()));

        return rPose;
    }

    /* Applies gaussian rotation noise to the provided quaternion. A new
     * quaternion object with added noise is returned. */
    public Quaternion applyQuaternionNoiseGaussian(Quaternion q){
        return AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(),
                getGaussian(AbstractLocaliser.getHeading(q), rotationNoise));
    }

    /* Applies gaussian noise to the provided point. A new point object with
     * the added noise is returned. */
    public Point applyPointNoiseGaussian(Point p){
        Point noisyPosition = messageFactory.newFromType(Point._TYPE);

        noisyPosition.setX(getGaussian(p.getX(), positionNoise));
        noisyPosition.setY(getGaussian(p.getY(), positionNoise));

        return noisyPosition;
    }

    /** Copies an entire pose array to store for later comparison */
    public PoseArray copyPoseArray(PoseArray poses) {
        PoseArray newPoses = messageFactory.newFromType(PoseArray._TYPE);
        List<Pose> newPoseList = newPoses.getPoses();

        for (Pose pose : poses.getPoses()) {
            newPoseList.add(copyPose(pose));
        }

        newPoses.setPoses(newPoseList);
        return newPoses;
    }

    public Pose copyPose(Pose pose){
        Pose newPose = messageFactory.newFromType(Pose._TYPE);

        newPose.getOrientation().setW(pose.getOrientation().getW());
        newPose.getOrientation().setX(pose.getOrientation().getX());
        newPose.getOrientation().setY(pose.getOrientation().getY());
        newPose.getOrientation().setZ(pose.getOrientation().getZ());

        newPose.getPosition().setX(pose.getPosition().getX());
        newPose.getPosition().setY(pose.getPosition().getY());
        newPose.getPosition().setZ(pose.getPosition().getZ());

        return newPose;
    }

    /* Check if one pose is equal to another. Will compare quaternion and
     * position data by checking the data that they contain, not the objects. */
    public static boolean poseIsEqual(Pose p1, Pose p2){
        // Make an initial assumption that poses are equal.
        return quaternionIsEqual(p1.getOrientation(), p2.getOrientation()) &&
                pointIsEqual(p1.getPosition(), p2.getPosition());
    }

    /* Check if two quaternions are equal. Checks not the object, but the data
     * that it contains. */
    public static boolean quaternionIsEqual(Quaternion q1, Quaternion q2){
        return q1.getW() == q2.getW() &&
                q1.getX() == q2.getX() &&
                q1.getY() == q2.getY() &&
                q1.getZ() == q2.getZ();
    }

    /* Check if two points are equal. Checks not the object, but the data that
     * it contains. */
    public static boolean pointIsEqual(Point p1, Point p2){
        return p1.getX() == p2.getX() &&
                p1.getY() == p2.getY() &&
                p1.getZ() == p2.getZ();
    }

    public static void printPose(Pose p){
        System.out.printf("[%f, %f], %f ", p.getPosition().getX(), p.getPosition().getY(), AbstractLocaliser.getHeading(p.getOrientation()));
    }

    public static String getTimeStampWithPose(PoseWithCovariance p, Time stamp){
        return stamp.secs + "."
                + stamp.nsecs + ","
                + p.getPose().getPosition().getX() + ","
                + p.getPose().getPosition().getY() + ","
                + AbstractLocaliser.getHeading(p.getPose().getOrientation());
    }

        public static String getTimeStampWithPose(Pose p, Time stamp){
        return stamp.secs + "."
                + stamp.nsecs + ","
                + p.getPosition().getX() + ","
                + p.getPosition().getY() + ","
                + AbstractLocaliser.getHeading(p.getOrientation());
    }

    public static boolean timeStampEqual(Time s1, Time s2){
        //System.out.println("s1: "+s1.secs+"."+s1.nsecs +
        //        "\t\tAND s2: " + s2.secs + "." + s2.nsecs +
        //        " AND secs.EQUAL=" +(s1.secs == s2.secs) +
        //        " AND nsecs.EQUAL="+(s1.nsecs == s2.nsecs));
        return s1.nsecs == s2.nsecs && s1.secs == s2.secs;
    }

    public static ArrayList<Pose> getRandomPoses(OccupancyGrid map, final int numToGet, Random randGen, MessageFactory messageFactory){
        final ChannelBuffer buff = map.getData();

        ArrayList<Pose> randomPoses = new ArrayList<Pose>();

        final int mapHeight = map.getInfo().getHeight();
        final int mapWidth = map.getInfo().getWidth();
        final float mapRes = map.getInfo().getResolution();

        for (int i = 0; i < numToGet; i++) {
            randomPoses.add(randomPose(mapWidth, mapHeight, mapRes, randGen, messageFactory, buff));
        }

        return randomPoses;

    }

    public static Pose randomPose(int mapWidth, int mapHeight, float mapRes, Random randGen, MessageFactory messageFactory, ChannelBuffer buff){
        boolean foundOpen = false;
        double randX = 0;
        double randY = 0;
        final int buffLength = buff.capacity();

        while (!foundOpen){
            randX = (randGen.nextDouble() * mapWidth);
            randY = (randGen.nextDouble() * mapHeight);

            // get the index in the array for this random point
            int index = SensorModel.getMapIndex((int) Math.round(randX), (int) Math.round(randY), (int) mapWidth, (int) mapHeight);
            if (index > 0 && index < buffLength){
                Byte cell = buff.getByte(index);
                if (cell.byteValue() == 0) {
                    // We are inside the map bounds and the cell is not occupied.
                    foundOpen = true;
                }
            }
        }

        Pose rPose = messageFactory.newFromType(Pose._TYPE);
        rPose.getPosition().setX(randX * mapRes);
        rPose.getPosition().setY(randY * mapRes);
//        System.out.println("random pose at " + randX + "," + randY + " generated");
        rPose.setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), LocalisationUtil.getGaussian(0, Math.PI, randGen)));

        return rPose;

    }

           public static Quaternion createQuaternion(MessageFactory factory) {
        Quaternion q = factory.newFromType(Quaternion._TYPE);

        // Set up 'identity' (blank) quaternion
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        return q;
    }

    public static Quaternion rotateQuaternion(Quaternion q_orig, double yaw, MessageFactory factory) {
        // Create a temporary Quaternion to represent the change in heading
        Quaternion q_headingChange = createQuaternion(factory);

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
        return multiply_quaternions(q_headingChange, q_orig, factory);
    }

    private static Quaternion multiply_quaternions(Quaternion qa, Quaternion qb, MessageFactory factory) {
        Quaternion combined = createQuaternion(factory);

        combined.setW(qa.getW() * qb.getW() - qa.getX() * qb.getX() - qa.getY() * qb.getY() - qa.getZ() * qb.getZ());
        combined.setX(qa.getX() * qb.getW() + qa.getW() * qb.getX() + qa.getY() * qb.getZ() - qa.getZ() * qb.getY());
        combined.setY(qa.getW() * qb.getY() - qa.getX() * qb.getZ() + qa.getY() * qb.getW() + qa.getZ() * qb.getX());
        combined.setZ(qa.getW() * qb.getZ() + qa.getX() * qb.getY() - qa.getY() * qb.getX() + qa.getZ() * qb.getW());
        return combined;

    }

    /*
     * The publisher passed in should be publishing to the initialpose topic.
     * theta is in degrees.
     */
    public static void publishInitialPose(double x, double y, double theta, Publisher<PoseWithCovarianceStamped> pub, MessageFactory factory) {
        PoseWithCovarianceStamped initialPose = pub.newMessage();
        initialPose.getPose().getPose().getPosition().setX(12.75);
        initialPose.getPose().getPose().getPosition().setY(4.075);
        initialPose.getPose().getPose().setOrientation(rotateQuaternion(createQuaternion(factory), Math.toRadians(128.92), factory));

        double[] cov = new double[36]; // 36 comes from the msg definition, and is fixed
        for (int i = 0; i < cov.length; i++) {
            cov[i] = 0.0;
        }
        initialPose.getPose().setCovariance(cov);

        initialPose.getHeader().setFrameId("/map");

        long timeNow = System.currentTimeMillis();

        while (System.currentTimeMillis() - timeNow < 1000) {
            pub.publish(initialPose);
        }
        System.out.println("Published intitial pose: X: " + initialPose.getPose().getPose().getPosition().getX()
                + ", Y: " + initialPose.getPose().getPose().getPosition().getY());
    }

}
