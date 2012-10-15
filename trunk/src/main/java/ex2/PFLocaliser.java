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
    public static final double CLUSTER_THRESHOLD = 0.7;

    public List<Pose> lastPoseList;
    public double[] poseWeights = new double[PARTICLE_NUMBER];
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

        poseWeights = weights; // Store the weights for all the poses this timestep.

        System.out.println("Mean weight: " + totalWeight/weights.length);

        // New set of poses which are sampled from the previous
        // set and have had noise added to them.
        ArrayList<Pose> sampledPoses = SamplingMethods.stackSample(poseList, weights, totalWeight, randGen);
        ArrayList<Pose> noisyPoses = util.applyNoise(sampledPoses);

        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(noisyPoses);

        lastPoseList = util.copyPoseArray(pa).getPoses();

        return pa;
    }

    @Override
    public Pose updatePose(PoseArray particleCloud) {
        return PoseEstimationMethods.clusterAverage(particleCloud, CLUSTER_THRESHOLD, messageFactory);
    }
}