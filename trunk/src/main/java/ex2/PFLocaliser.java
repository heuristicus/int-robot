package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.ros.node.topic.Publisher;
import pf.AbstractLocaliser;
import pf.SensorModel;
import sensor_msgs.LaserScan;

public class PFLocaliser extends AbstractLocaliser {

    public static final int PARTICLE_NUMBER = 100;
    public static final double ROTATION_NOISE = Math.PI/30.0;
    public static final double POSITION_NOISE = 0.15;
    
    // How far another point is before being considered a clustered point
    public static final double CLUSTER_THRESHOLD = 0.5;
    public List<Pose> lastPoseList;

    Publisher<Pose> expPub;

    private Random randGen = new Random();

    public PFLocaliser() {
        super();
    }

    

    @Override
    public PoseArray initialisePF(PoseWithCovarianceStamped initialPose) {
//        System.out.println(AbstractLocaliser.getHeading(initialPose.getPose().getPose().getOrientation())
//                + "   x " + initialPose.getPose().getPose().getPosition().getX()
//                + "   y " + initialPose.getPose().getPose().getPosition().getY());
//
//        Pose p = messageFactory.newFromType(Pose._TYPE);
//        p.setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 2.251));
//        Point pt = messageFactory.newFromType(Point._TYPE);
//        pt.setX(-3.80);
//        pt.setY(-11.90);
//        p.setPosition(pt);
        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        ArrayList<Pose> poseList = new ArrayList();

//        for (int i= 0; i < PARTICLE_NUMBER; i++) {
//            Pose poseWithNoise = applyNoise(p);
//            poseList.add(poseWithNoise);
//        }

        for (int i= 0; i < PARTICLE_NUMBER; i++) {
            Pose poseWithNoise = applyNoise(initialPose.getPose().getPose());
            poseList.add(poseWithNoise);
        }
        
        pa.setPoses(poseList);
        return pa;
    }

    @Override
    /** Called every time there is a laser scan reading. */
    public PoseArray updateParticleCloud(LaserScan scan, OccupancyGrid map, PoseArray particleCloud) {
        final int readings = scan.getRanges().length;

        // If we haven't moved since last call of this method,
        // don't modify the particle cloud.
        if (lastPoseList != null){
            List<Pose> newCloud = particleCloud.getPoses();

            boolean allEqual = true;
            for (int i = 0; i < lastPoseList.size(); i++) {
//                Pose oldPose = lastPoseList.get(i);
//                Pose newPose = newCloud.get(i);
                boolean equal = poseIsEqual(lastPoseList.get(i), newCloud.get(i));

//                System.out.print(oldPose.getPosition().getX()+", "+oldPose.getPosition().getY()+"\t");
//                System.out.println(newPose.getPosition().getX()+", "+newPose.getPosition().getY());

                /*
                 * Save time by jumping out of the loop as soon as the is a
                 * pose which differs. Do we really need this? If the robot has
                 * moved at all, all points will have changed, so we only need
                 * to sample a single pose rather than all of them, assuming that
                 * robot movements are being applied to all points in the method
                 * that is hidden from us.
                 */
                if (! equal) {
                    allEqual = false;
                    break;
                }
            }

            /*
             * If all the points in the list are the same, make a deep copy of
             * the new pose array and store it. This is also probably unnecessary.
             * We could just keep the same lastPoseList until the particleCloud
             * changes, and only then copy it to make a new array - this would
             * save a significant amount of time if we are using a large number
             * of particles.
             */
            if (allEqual) {
                lastPoseList = copyPoseArray(particleCloud).getPoses();
                return particleCloud;
            }
        }
        System.out.println("Robot movement detected. Adding noise lolz "+System.currentTimeMillis()%10);

        List<Pose> poseList = particleCloud.getPoses();
        double[] weights = new double[poseList.size()];
        
        double totalWeight = 0.0;
        double weight;
        for (int i = 0; i < weights.length; i++) {
            weight = SensorModel.getWeight(scan, map, poseList.get(i), readings);
            totalWeight += weight;
            weights[i] = weight;
        }

        // New set of poses which are sampled from the previous
        // set and have had noise added to them.
        ArrayList<Pose> newPoses = new ArrayList(poseList.size());
        double counter;
        for (int i = 0; i < poseList.size(); i++) {
            counter = 0.0;
            for (int j = 0; j < weights.length; j++) {
                double rand = randGen.nextDouble() * totalWeight;

                counter += weights[j];
                if (rand < counter) {
                    Pose nextPose = poseList.get(j);
                    /*
                     * Do we really want to apply the same amount of noise to
                     * particles on rotation and movement? If the action only
                     * rotates the robot, the amount of noise applied to
                     * the position of the particles should be smaller. Is this
                     * true in the opposite case as well? (probably not)
                     */
                    Pose nextPoseWithNoise = applyNoise(nextPose);
                    newPoses.add(nextPoseWithNoise);
                    break;
                }
            }
        }
        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(newPoses);
        lastPoseList = copyPoseArray(pa).getPoses();

        return pa;

    }

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
     * position data by checking the data that they contain, not the objects.
     */
    public boolean poseIsEqual(Pose p1, Pose p2){
        // Make an initial assumption that poses are equal.
        return quaternionIsEqual(p1.getOrientation(), p2.getOrientation()) &&
                pointIsEqual(p1.getPosition(), p2.getPosition());
    }

    /* Check if two quaternions are equal. Checks not the object, but the data
     * that it contains.
     */
    public boolean quaternionIsEqual(Quaternion q1, Quaternion q2){
        return q1.getW() == q2.getW() &&
                q1.getX() == q2.getX() &&
                q1.getY() == q2.getY() &&
                q1.getZ() == q2.getZ();
    }

    /* Check if two points are equal. Checks not the object, but the data that
     * it contains.
     */
    public boolean pointIsEqual(Point p1, Point p2){
        return p1.getX() == p2.getX() &&
                p1.getY() == p2.getY() &&
                p1.getZ() == p2.getZ();
    }

    @Override
    public Pose updatePose(PoseArray particleCloud) {
        List<Pose> poses = particleCloud.getPoses();
        int[] poseClusterWeight = new int[poses.size()]; // How clustered this value is

        for (int i = 0; i < poses.size(); i++) {
            Pose pose = poses.get(i);

            for (int j = 0; j < poses.size(); j++) {
                Pose otherPose = poses.get(j);
                double distance = Point2D.distance(
                        pose.getPosition().getX(),
                        pose.getPosition().getY(),
                        otherPose.getPosition().getX(),
                        otherPose.getPosition().getY());
                if (distance <= CLUSTER_THRESHOLD) {
                    poseClusterWeight[i]++;
                }
            }
        }

        int biggestClusterWeight = 0;
        int biggestClusterIndex = 0;
        for (int i = 0; i < poseClusterWeight.length; i++) {
            if (poseClusterWeight[i] > biggestClusterWeight) {
                biggestClusterWeight = poseClusterWeight[i];
                biggestClusterIndex = i;
            }
        }
        //System.out.println("location: [" + poses.get(biggestClusterIndex).getPosition().getX()
          //      + ", " + poses.get(biggestClusterIndex).getPosition().getY()
          //      + "], heading: " + AbstractLocaliser.getHeading(poses.get(biggestClusterIndex).getOrientation()));
        return poses.get(biggestClusterIndex);
    }

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
     * quaternion object with added noise is returned.
     */
    public Quaternion applyQuaternionNoiseGaussian(Quaternion q){
        return AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 
                getGaussian(getHeading(q), ROTATION_NOISE));
    }

    /* Applies gaussian noise to the provided point. A new point object with
     * the added noise is returned.
     */
    public Point applyPointNoiseGaussian(Point p){
        Point noisyPosition = messageFactory.newFromType(Point._TYPE);
        
        noisyPosition.setX(getGaussian(p.getX(), POSITION_NOISE));
        noisyPosition.setY(getGaussian(p.getY(), POSITION_NOISE));

        return noisyPosition;
    }

}
