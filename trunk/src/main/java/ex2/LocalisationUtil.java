package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.Quaternion;
import java.util.List;
import java.util.Random;
import org.ros.message.MessageFactory;
import pf.AbstractLocaliser;

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

    /** Calculates a random value under a distribution specified by the given
     * mean and standard deviation */
    public double getGaussian(double mean, double stdDev) {
        double rand = randGen.nextGaussian();
        rand *= stdDev;
        rand += mean;
        return rand;
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
    public boolean poseIsEqual(Pose p1, Pose p2){
        // Make an initial assumption that poses are equal.
        return quaternionIsEqual(p1.getOrientation(), p2.getOrientation()) &&
                pointIsEqual(p1.getPosition(), p2.getPosition());
    }

    /* Check if two quaternions are equal. Checks not the object, but the data
     * that it contains. */
    public boolean quaternionIsEqual(Quaternion q1, Quaternion q2){
        return q1.getW() == q2.getW() &&
                q1.getX() == q2.getX() &&
                q1.getY() == q2.getY() &&
                q1.getZ() == q2.getZ();
    }

    /* Check if two points are equal. Checks not the object, but the data that
     * it contains. */
    public boolean pointIsEqual(Point p1, Point p2){
        return p1.getX() == p2.getX() &&
                p1.getY() == p2.getY() &&
                p1.getZ() == p2.getZ();
    }
}
