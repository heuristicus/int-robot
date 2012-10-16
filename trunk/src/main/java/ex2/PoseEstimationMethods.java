package ex2;

import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import org.ros.message.MessageFactory;
import pf.AbstractLocaliser;

public class PoseEstimationMethods {

    public static Pose clusterAverage(PoseArray particleCloud, double clusterThreshold, MessageFactory messageFactory){
        List<Pose> poses = particleCloud.getPoses();
        ArrayList<Pose> connectedParticles = new ArrayList<Pose>();
        int highestConnectivity = 0;

        for (int i = 0; i < poses.size(); i++) {
            Pose pose = poses.get(i);
            double curX = pose.getPosition().getX();
            double curY = pose.getPosition().getY();
            ArrayList<Pose> curConnected = new ArrayList<Pose>();
            curConnected.add(pose);
            int currentConnectivity = 0;
            for (int j = 0; j < poses.size(); j++) {
                Pose otherPose = poses.get(j);
                double distance = Point2D.distance(
                        curX,
                        curY,
                        otherPose.getPosition().getX(),
                        otherPose.getPosition().getY());
                if (distance <= clusterThreshold){
                    currentConnectivity++;
                    curConnected.add(otherPose);
                }
            }
            if (currentConnectivity > highestConnectivity){
                highestConnectivity = currentConnectivity;
                connectedParticles = curConnected;
            }
        }
        return averagePose(connectedParticles, messageFactory);
    }

    private static Pose averagePose(ArrayList<Pose> poses, MessageFactory messageFactory) {
        double avgx = 0;
        double avgy = 0;
        double avgrot = 0;

        for (int i= 1; i <= poses.size(); i++) {
            Pose cur = poses.get(i - 1);
            avgx += (cur.getPosition().getX() - avgx)/i;
            avgy += (cur.getPosition().getY() - avgy)/i;
            avgrot += (AbstractLocaliser.getHeading(cur.getOrientation()) - avgrot);
        }

        Pose avgPose = messageFactory.newFromType(Pose._TYPE);
        avgPose.getPosition().setX(avgx);
        avgPose.getPosition().setY(avgy);
        avgPose.setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), avgrot));

        return avgPose;
    }

    public static Pose highestWeighted(PoseArray particleCloud, double[] weights){
        double maxWeight = 0;
        int maxWeightIndex = 0;
        for (int i = 0; i < weights.length; i++) {
            if (weights[i] > maxWeight) {
                maxWeight = weights[i];
                maxWeightIndex = i;
            }
        }
        return particleCloud.getPoses().get(maxWeightIndex);
    }

    public static Pose mostConnectedPointClustering(PoseArray particleCloud, double clusterThreshold){
        List<Pose> poses = particleCloud.getPoses();
        int highestConnectivity = 0;
        int highestConnectedParticleIndex = 0;

        for (int i = 0; i < poses.size(); i++) {
            Pose pose = poses.get(i);
            int currentClusterConnectivity = 0;
            for (int j = 0; j < poses.size(); j++) {
                Pose otherPose = poses.get(j);
                double distance = Point2D.distance(
                        pose.getPosition().getX(),
                        pose.getPosition().getY(),
                        otherPose.getPosition().getX(),
                        otherPose.getPosition().getY());
                if (distance <= clusterThreshold) {
                    currentClusterConnectivity++;
                }
            }
            if (currentClusterConnectivity > highestConnectivity){
                highestConnectivity = currentClusterConnectivity;
                highestConnectedParticleIndex = i;
            }
        }

        Pose estimatedPose = poses.get(highestConnectedParticleIndex);

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
