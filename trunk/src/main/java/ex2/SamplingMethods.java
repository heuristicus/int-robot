package ex2;

import geometry_msgs.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SamplingMethods {

    public static ArrayList<Pose> lowVarianceSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
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

    public static ArrayList<Pose> lowVarianceSample(List<Pose> poseList, double[] weights, double totalWeight, int numberOfSamples, Random randGen){
        // Equation source: Probabilistic Robotics (Thrun 2005)
        // M = totalNumOfWeights
        // m = 1 to M (iteration counter)
        // Get a random number 'r' between 0 and M^-1
        // s = r
        // On every iteration, s += M^-1
        // If s >= totalWeightSoFar
        //     then pick the particle with index curParticle
        //     else loop through weights, accumulating total until s >= totalWeightSoFar

        final int M = numberOfSamples;
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

    public static ArrayList<Pose> stratifiedSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
	// boundary between two groups
	int boundary = weights.length/2;
	double groupOneTotal = 0.0;
	double groupTwoTotal = 0.0;
	
	// calculate weight distribution in the two groups
	for (int i = 0; i < weights.length; i++){
	    if (i < boundary){
		groupOneTotal += weights[i];
	    } else {
		groupTwoTotal += weights[i];
	    }
	}
	
	// Give the groups a proportional weight between 0 and 1
	double groupOneWeight = groupOneTotal / totalweight;
	double groupTwoWeight = groupTwoTotal / totalweight;
		
	int groupOneParticleNumber = groupOneWeight * weights.length;
	int groupTwoParticleNumber = weights.length - groupOneParticleNumber;
	
	ArrayList<Pose> newPoses = new ArrayList<Pose>(lowVarianceSample(poseList, weights, totalWeight, groupOneParticleNumber, Random randGen));
	newPoses.addAll(lowVarianceSample(poseList, weights, totalWeight, groupTwoParticleNumber, Random randGen));
	

        return newPoses;
    }

    public static ArrayList<Pose> stackSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
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

}
