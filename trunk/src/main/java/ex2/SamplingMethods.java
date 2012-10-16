package ex2;

import geometry_msgs.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.ros.message.MessageFactory;
import pf.AbstractLocaliser;

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
        int curWeight = 0;

        ArrayList<Pose> newPoses = new ArrayList<Pose>(M);
        for (int m = 0; m < M; m++) {
            System.out.println("s: " + s + " totalweight: " + totalWeightSoFar);
            while (s > totalWeightSoFar) {
                totalWeightSoFar += weights[curWeight];
                curParticle = curWeight;
                curWeight++;
                System.out.println("totalweight now: " + totalWeightSoFar);
                //curParticle++;
            }
            System.out.println("adding curparticle: " + curParticle);
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

    public static ArrayList<Pose> lowVarianceSubSample(
            List<Pose> poseList, double[] weights, double totalWeight,
            int startIndex, int numberOfParticles, Random randGen){
        // Parameterised version of the lowVarianceSample method above.
        // This is to be used as a sub-sampling method for stratified sampling

        System.out.println("LOW VARIANCE PARAMS: " +
                "Weights.length: "+weights.length+
                "\ttotalWeight: "+totalWeight+
                "\tstartIndex: "+startIndex+
                "\tnumOfParticles: "+numberOfParticles);

        final int M = numberOfParticles;
        final double stepSize = totalWeight / M;
        final double r = randGen.nextDouble() * stepSize;
        double s = r;

        double totalWeightSoFar = weights[startIndex];
        int curParticle = startIndex;
        int curWeight = startIndex;

        ArrayList<Pose> newPoses = new ArrayList<Pose>(M);
        for (int m = 0; m < M; m++) {
        //    System.out.println("s: " + s + " totalweight: " + totalWeightSoFar + " m: "+m);
            while (s > totalWeightSoFar && curWeight < weights.length) {
//                System.out.println("totalweight now: " + totalWeightSoFar);
  //              System.out.println("s is : " + s + "curweight " + curWeight);
                totalWeightSoFar += weights[curWeight];
                curParticle = curWeight;
                curWeight++;
    //            System.out.println("totalweight now: " + totalWeightSoFar);
            }
      //      System.out.println("adding curparticle: " + curParticle);
            Pose nextPose = poseList.get(curParticle);
            newPoses.add(nextPose);
            s += stepSize;
        }

//        System.out.println("curweight: " + curWeight);

        if (newPoses.size() != numberOfParticles) {
            System.out.println("WARNINGS! ALL OF THE WARNINGS! LOW VARIANCE "
                    + "SAMPLING MUST BE BROKEN. NEW POSES: "+newPoses.size()+
                    " AND NUMBEROFPARTICLES: "+numberOfParticles);
        }

        return newPoses;
    }

    public static ArrayList<Pose> stratifiedSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
	// boundary between two groups
	int boundary = weights.length/2;
	double groupOneTotal = 0.0;
	double groupTwoTotal = 0.0;
	
	// calculate weight distribution in the two groups
	/*for (int i = 0; i < weights.length; i++){
	    if (i < boundary){
		groupOneTotal += weights[i];
	    } else {
		groupTwoTotal += weights[i];
	    }
	}*/ // Optimised version below instead

        // calculate weight distribution in the two groups
        for (int i = 0; i < boundary; i++) {
            groupOneTotal += weights[i];
        }
        for (int i = boundary; i < weights.length; i++) {
            groupTwoTotal += weights[i];
        }

	// Give the groups a proportional weight between 0 and 1
	double groupOneProportion = groupOneTotal / totalWeight;
		
	int groupOneParticleNumber = (int) Math.round(groupOneProportion * (double)weights.length);
	int groupTwoParticleNumber = weights.length - groupOneParticleNumber;
	
	ArrayList<Pose> newPoses = new ArrayList<Pose>(lowVarianceSubSample(poseList, weights, groupOneTotal, 0, groupOneParticleNumber, randGen));
	newPoses.addAll(lowVarianceSubSample(poseList, weights, groupTwoTotal, groupOneParticleNumber, groupTwoParticleNumber, randGen));
	
        return newPoses;
    }

    public static Pose singleStackSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
        double rand = randGen.nextDouble() * totalWeight;
        double counter = 0.0;
        int i;
        for (i = 0; i < weights.length; i++) {
            counter += weights[i];
            if (rand < counter){
                break;
            }
        }
        LocalisationUtil.printPose(poseList.get(i));
        return poseList.get(i);
    }

    public static ArrayList<Pose> stackSample(List<Pose> poseList, double[] weights, double totalWeight, Random randGen){
        ArrayList<Pose> newPoses = new ArrayList(poseList.size());
        double counter;
        for (int i = 0; i < poseList.size(); i++) {
            counter = 0.0;
            double rand = randGen.nextDouble() * totalWeight;
            for (int j = 0; j < weights.length; j++) {
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
