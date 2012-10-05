package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import pf.AbstractLocaliser;
import pf.SensorModel;
import sensor_msgs.LaserScan;

public class PFLocaliser extends AbstractLocaliser {

    public static final int PARTICLE_NUMBER = 3;
    public static final double ROTATION_NOISE = Math.PI/30.0;
    public static final double POSITION_NOISE = 0.15;
    
    // How far another point is before being considered a clustered point
    public static final double CLUSTER_THRESHOLD = 0.5;
    public List<Pose> lastPoseList;

    private Random randGen = new Random();

    public PFLocaliser() {
        super();
    }

    @Override
    public PoseArray initialisePF(PoseWithCovarianceStamped initialPose) {
        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        ArrayList<Pose> poseList = new ArrayList();

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
                Pose oldPose = lastPoseList.get(i);
                Pose newPose = newCloud.get(i);
                boolean equal = oldPose.getPosition().getX() == newPose.getPosition().getX();
                equal = equal && (oldPose.getPosition().getY() == newPose.getPosition().getY());

                System.out.print(oldPose.getPosition().getX()+", "+oldPose.getPosition().getY()+"\t");
                System.out.println(newPose.getPosition().getX()+", "+newPose.getPosition().getY());

                if (! equal) {
                    allEqual = false;
                    break;
                }

            }
            if (allEqual) {
                PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
                pa.setPoses(particleCloud.getPoses());
                System.out.println("Robot hasn't moved. Exiting "+System.currentTimeMillis()%10);
                lastPoseList = pa.getPoses();
                return pa;
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
                    Pose nextPoseWithNoise = applyNoise(nextPose);
                    newPoses.add(nextPoseWithNoise);
                    break;
                }
            }
        }
        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(newPoses);
        lastPoseList = pa.getPoses();
        return pa;
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
        Quaternion orient = pose.getOrientation();
        Point pos = pose.getPosition();

        Point rPos = messageFactory.newFromType(Point._TYPE);
        rPos.setX(getGaussian(pos.getX(), POSITION_NOISE));
        rPos.setY(getGaussian(pos.getY(), POSITION_NOISE));
        //Quaternion rOrient = rotateQuaternion(orient,
        //      getGaussian(AbstractLocaliser.getHeading(orient), ROTATION_NOISE));

        Pose rPose = messageFactory.newFromType(Pose._TYPE);
//        rPose.setOrientation(rOrient);
        rPose.setOrientation(orient);
        rPose.setPosition(rPos);
        return rPose;
    }
}
