package ex2;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import pf.AbstractLocaliser;
import pf.SensorModel;
import sensor_msgs.LaserScan;

public class PFLocaliser extends AbstractLocaliser {

    public static final int PARTICLE_NUMBER = 100;
    public static final double ROTATION_NOISE = Math.PI/30.0;
    public static final double POSITION_NOISE = 0.15;
    
    // How far another point is before being considered a clustered point
    public static final double CLUSTER_THRESHOLD = 0.7;

    public List<Pose> lastPoseList;
    public double[] poseWeights = new double[PARTICLE_NUMBER];
    private Random randGen = new Random();
    private LocalisationUtil util;

    public double wslow = 0.0;
    public double wfast = 0.0;
    public double decayParamSlow = 0.01;
    public double decayParamFast = 0.75;
    public int randomParticleThreshold = (int) (PARTICLE_NUMBER * 0.6);
    public boolean firstLoop = true;


    public double particleMinWeightThreshold = 50.0;
    public double cloudMinMeanWeightThreshold = 80.0;

    public PFLocaliser() {
        super();
        util = new LocalisationUtil(messageFactory, randGen,
                POSITION_NOISE, ROTATION_NOISE);
        System.out.println("pflocaliser");
    }

    @Override
    public PoseArray initialisePF(PoseWithCovarianceStamped initialPose) {
        if (experimentMode) {
            if (realWorldMode) {
                return realExperimentStart(initialPose);
            } else {
                return experimentStart(initialPose);
            }
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

    public PoseArray realExperimentStart(PoseWithCovarianceStamped initialPose){
        System.out.println("Experiment start");
        Quaternion q = AbstractLocaliser.createQuaternion();
        q.setW(0.40);
        q.setZ(0.92);
        System.out.println(AbstractLocaliser.getHeading(q));
        System.out.println(AbstractLocaliser.getHeading(initialPose.getPose().getPose().getOrientation())
                + "   x " + initialPose.getPose().getPose().getPosition().getX()
                + "   y " + initialPose.getPose().getPose().getPosition().getY());

        Pose p = messageFactory.newFromType(Pose._TYPE);
        p.setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 2.321)); //approximate
        Point pt = messageFactory.newFromType(Point._TYPE);
        pt.setX(28.39);
        pt.setY(19.39);
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
        // if running the augmented algorithm, use the update method for that instead
        if (augmented) {
            return augmented_updateParticleCloud(scan, map, particleCloud);
        }
        
        final int readings = scan.getRanges().length;

        // If we haven't moved since last call of this method,
        // don't modify the particle cloud.
        if (lastPoseList != null){
            List<Pose> newCloud = particleCloud.getPoses();

            // THIS CAUSES SLOWDOWN?
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

        System.out.println("Mean weight: " + totalWeight/weights.length);

        // New set of poses which are sampled from the previous
        // set and have had noise added to them.
 //       ArrayList<Pose> sampledPoses = SamplingMethods.stackSample(poseList, weights, totalWeight, randGen);
       ArrayList<Pose> sampledPoses = SamplingMethods.lowVarianceSample(poseList, weights, totalWeight, randGen);
//        ArrayList<Pose> sampledPoses = SamplingMethods.stratifiedSample(poseList, weights, totalWeight, randGen);

        List<Pose> noisyPoses = util.applyNoise(sampledPoses);

        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(noisyPoses);

        lastPoseList = util.copyPoseArray(pa).getPoses();

        return pa;
    }

    public PoseArray augmented_updateParticleCloud(LaserScan scan, OccupancyGrid map, PoseArray particleCloud){
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
        double weightAvg = 0.0;
        for (int i = 0; i < weights.length; i++) {
            weight = SensorModel.getWeight(scan, map, poseList.get(i), readings);
            totalWeight += weight;
            weights[i] = weight;
            weightAvg += (1.0/weights.length) * weight;
        }


        ArrayList<Pose> sampledPoses = adaptiveLowVariance(map, poseList, weights, totalWeight, weightAvg);
//        List<Pose> sampledPoses = particleThreshold(map, poseList, weights, totalWeight, weightAvg);
//       ArrayList<Pose> sampledPoses = cloudThreshold(map, poseList, weights, totalWeight, weightAvg);

        List<Pose> noisyPoses = util.applyNoise(sampledPoses);

        PoseArray pa = messageFactory.newFromType(PoseArray._TYPE);
        pa.setPoses(noisyPoses);

        lastPoseList = util.copyPoseArray(pa).getPoses();

        return pa;
    }

    public List<Pose> particleThreshold(OccupancyGrid map, List<Pose> poseList, double[] weights, double totalWeight, double weightAvg){
        if (weightAvg > cloudMinMeanWeightThreshold) {
            return poseList;
        }

        ArrayList<Pose> sampledPoses = new ArrayList<Pose>();

        final ChannelBuffer buff = map.getData();

        final int mapHeight = map.getInfo().getHeight();
        final int mapWidth = map.getInfo().getWidth();
        final float mapRes = map.getInfo().getResolution();

        for (int i= 0; i < weights.length; i++) {
            if (weights[i] < particleMinWeightThreshold && randGen.nextDouble() < 0.25) {
                System.out.println("ParticleWeight is "+weights[i]+" thus random");
                sampledPoses.add(util.randomPose(mapWidth, mapHeight, mapRes, randGen, messageFactory, buff));
            //} else {
            //    sampledPoses.add(poseList.get(i));
            }
        }

        ArrayList<Pose> lowVarianceSampledPoses =
                SamplingMethods.lowVarianceSubSample(poseList, weights,
                totalWeight, 0, weights.length-sampledPoses.size(), randGen);
        sampledPoses.addAll(lowVarianceSampledPoses);

        return sampledPoses;

    }

    public ArrayList<Pose> cloudThreshold(OccupancyGrid map, List<Pose> poseList, double[] weights, double totalWeight, double weightavg){
        ArrayList<Pose> sampledPoses = new ArrayList<Pose>();

        int randomParticles = 0;

        if (weightavg < cloudMinMeanWeightThreshold){
            randomParticles = (int) (poseList.size() * randomParticleThreshold);
            sampledPoses.addAll(util.getRandomPoses(map, randomParticles, randGen, messageFactory));
        }

        sampledPoses = SamplingMethods.lowVarianceSubSample(poseList, weights, totalWeight, poseList.size() - randomParticles, randomParticleThreshold, randGen);

        return sampledPoses;

    }

    public ArrayList<Pose> adaptiveLowVariance(OccupancyGrid map, List<Pose> poseList, double[] weights, double totalWeight, double weightavg){

        if (firstLoop){
            wslow = weightavg;
            wfast = weightavg;
            firstLoop = false;
        }

        System.out.println("weightavg: " + weightavg);
        wslow += decayParamSlow * (weightavg - wslow);
        wfast += decayParamFast * (weightavg - wfast);

        System.out.println("wslow: " + wslow + " wfast: " + wfast + "  1.0-wfast/wslow = " + (1.0 - (wfast/wslow)));

        int randomPoses = 0;
        // If wfast is smaller than wslow, pRand is high. If wfast ~= wslow, pRand is small.
        double pRand = 1.0 - wfast / wslow;

        for (int i= 0; i < poseList.size() && randomPoses <= randomParticleThreshold; i++) {
            // Generate a random pose with probability pRand
            if (randGen.nextDouble() < pRand){
                randomPoses++;
            }
        }


        // Get the required number of random poses.
        ArrayList<Pose> sampledPoses = LocalisationUtil.getRandomPoses(map, randomPoses, randGen, messageFactory);
        // The rest of the particles are sampled normally using low variance sampling.
        sampledPoses.addAll(SamplingMethods.lowVarianceSubSample(poseList, weights, totalWeight, 0, poseList.size() - randomPoses, randGen));

        System.out.println("Sampled " + randomPoses + " random poses and " + (poseList.size() - randomPoses) + " normal poses.");
        
        return sampledPoses;
    }

    @Override
    public Pose updatePose(PoseArray particleCloud) {
        return PoseEstimationMethods.clusterAverage(particleCloud, CLUSTER_THRESHOLD, messageFactory);

    }
}
