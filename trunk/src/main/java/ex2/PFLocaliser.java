package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
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
    public static final double ROTATION_NOISE = Math.PI/30;
    public static final double POSITION_NOISE = 0.15;
    
    // How far another point is before being considered a clustered point
    public static final double CLUSTER_THRESHOLD = 0.9;

    public List<Pose> lastPoseList;

    private Random randGen = new Random();
    private LocalisationUtil util;

    public PFLocaliser() {
        super();
        util = new LocalisationUtil(messageFactory, randGen,
                POSITION_NOISE, ROTATION_NOISE);
        System.out.println("pflocaliser");
    }

    @Override
    public PoseArray initialisePF(PoseWithCovarianceStamped initialPose) {
        if (experimentMode) {
            return experimentStart(initialPose);
        }

        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        ArrayList<Pose> poseList = new ArrayList();

        for (int i= 0; i < PARTICLE_NUMBER; i++) {
            Pose poseWithNoise = util.applyNoise(initialPose.getPose().getPose());
            poseList.add(poseWithNoise);
        }

        pa.setPoses(poseList);
        return pa;
    }

    public PoseArray experimentStart(PoseWithCovarianceStamped initialPose){
        System.out.println("Experiment start");
        System.out.println(AbstractLocaliser.getHeading(initialPose.getPose().getPose().getOrientation())
                + "   x " + initialPose.getPose().getPose().getPosition().getX()
                + "   y " + initialPose.getPose().getPose().getPosition().getY());

        Pose p = messageFactory.newFromType(Pose._TYPE);
        p.setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 2.251));
        Point pt = messageFactory.newFromType(Point._TYPE);
        pt.setX(12.75);
        pt.setY(4.075);
        p.setPosition(pt);
        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        ArrayList<Pose> poseList = new ArrayList();

        for (int i= 0; i < PARTICLE_NUMBER; i++) {
            Pose poseWithNoise = util.applyNoise(p);
            poseList.add(poseWithNoise);
        }

        pa.setPoses(poseList);
        System.out.println("experiment started");
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
                boolean equal = LocalisationUtil.poseIsEqual(lastPoseList.get(i), newCloud.get(i));

                if (! equal) {
                    allEqual = false;
                    break;
                }
            }

            if (allEqual) {
                return particleCloud;
            }
        }

        List<Pose> poseList = particleCloud.getPoses();
        double[] weights = new double[poseList.size()];
        
        double totalWeight = 0.0;
        double weight;
        for (int i = 0; i < weights.length; i++) {
            weight = SensorModel.getWeight(scan, map, poseList.get(i), readings);
            totalWeight += weight;
            weights[i] = weight;
        }

//        System.out.println("Mean weight: " + totalWeight/weights.length);

        // New set of poses which are sampled from the previous
        // set and have had noise added to them.
        ArrayList<Pose> sampledPoses = stackSample(poseList, weights, totalWeight);
        ArrayList<Pose> noisyPoses = util.applyNoise(sampledPoses);

        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(noisyPoses);

        lastPoseList = util.copyPoseArray(pa).getPoses();

        return pa;
    }

    public ArrayList<Pose> lowVarianceSample(List<Pose> poseList, double[] weights, double totalWeight){
        // Equation source: Probabilistic Robotics (Thrun 2005)
        // M = totalNumOfWeights
        // m = 1 to M (iteration counter)
        // Get a random number 'r' between 0 and M^-1
        // s = r
        // On every iteration, s += M^-1
        // If s >= totalWeightSoFar
        //     then pick the particle with index curParticle
        //     else loop through weights, accumulating total until s >= totalWeightSoFar

        final int M = poseList.size();
        final double stepSize = totalWeight / M;
        final double r = randGen.nextDouble() * stepSize;
        double s = r;

        double totalWeightSoFar = 0.0;
        int curParticle = 0;

        ArrayList<Pose> newPoses = new ArrayList<Pose>(M);
        for (int m = 0; m < M; m++) {
            while (s > totalWeightSoFar) {
                totalWeightSoFar += weights[curParticle];
                curParticle++;
            }
            Pose nextPose = poseList.get(curParticle);
            newPoses.add(nextPose);
            s += stepSize;
        }

        if (newPoses.size() != poseList.size()) {
            System.out.println("WARNINGS! ALL OF THE WARNINGS! LOW VARIANCE "
                    + "SAMPLING MUST BE BROKEN. NEW POSES: "+newPoses.size()+
                    " AND POSELIST: "+poseList.size());
        }

        return newPoses;
    }

    public ArrayList<Pose> stackSample(List<Pose> poseList, double[] weights, double totalWeight){
        ArrayList<Pose> newPoses = new ArrayList(poseList.size());
        double counter;
        for (int i = 0; i < poseList.size(); i++) {
            counter = 0.0;
            for (int j = 0; j < weights.length; j++) {
                double rand = randGen.nextDouble() * totalWeight;

                counter += weights[j];
                if (rand < counter) {
                    Pose nextPose = poseList.get(j);
                    /* Do we really want to apply the same amount of noise to
                     * particles on rotation and movement? If the action only
                     * rotates the robot, the amount of noise applied to
                     * the position of the particles should be smaller. Is this
                     * true in the opposite case as well? (probably not) */
                    newPoses.add(nextPose);
                    break;
                }
            }
        }
        return newPoses;
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

        Pose estimatedPose = poses.get(biggestClusterIndex);

        // Publish estimatedPose so it can be viewed in rviz
//        Publisher<PoseStamped> posePub = node.newPublisher("estimated_pose", PoseStamped._TYPE);
//        PoseStamped pubPose = posePub.newMessage();
//        pubPose.setPose(estimatedPose);
//        pubPose.setHeader(particleCloud.getHeader());
//        posePub.publish(pubPose);
//        System.out.println("location: [" + poses.get(biggestClusterIndex).getPosition().getX()
  //            + ", " + poses.get(biggestClusterIndex).getPosition().getY()
    //          + "], heading: " + AbstractLocaliser.getHeading(poses.get(biggestClusterIndex).getOrientation()) + "\n");
        return estimatedPose;
    }
}
