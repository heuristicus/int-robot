/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import ex3.PRM;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author HammyG
 */
public class MainNode extends MainNodeAbstract {

    private enum Phase {

        FINDINGROOM,
        EXPLORING,
        PRMTOPERSON,
        PRMTOROOM,
        INMEETINGROOM,
        COMPLETED
    }
    private float[] lastCameraData;
    private Pose lastEstimatedPose;
    private PoseStamped meetingRoomLocation = null;
    private Phase currentPhase = Phase.FINDINGROOM;
    private int pplCount = 0;
    private int targetPplCount = 3;

    public void Start() {
        meetingRoomLocation = messageFactory.newFromType(PoseStamped._TYPE);
        while (meetingRoomLocation == null) {
            //find room
        }
        while (currentPhase != Phase.COMPLETED) {
            currentPhase = Phase.EXPLORING;
            RectangleWithDepth areaOfPerson = findPerson();
            double areaCenterX = areaOfPerson.getCenterX();
            double fromCenterX = areaCenterX - (CAMERA_DIMENSIONS.width / 2);
            double turnAngle = Math.toRadians(10);
            while (Math.abs(fromCenterX) > 10) {
                if (fromCenterX > 0) {
                    //rectangle on the right
                    driver.turn(-turnAngle, true, true);
                } else {
                    //rectangle on left
                    driver.turn(turnAngle, true, true);
                }
                areaOfPerson = findPerson();
            }
            currentPhase = Phase.PRMTOPERSON;
            Pose estimatedPoseCopy = StaticMethods.copyPose(lastEstimatedPose);
            PoseStamped personLocation = getPersonLocation(estimatedPoseCopy, areaOfPerson);

            setPRMGoal(personLocation);
            if (personLost()) {
                continue;
            }
            if (personAcceptsInvite()) {
                currentPhase = Phase.PRMTOROOM;
                setPRMGoal(meetingRoomLocation);
                pplCount++;
                currentPhase = Phase.INMEETINGROOM;
                if (isTaskComplete()) {
                    currentPhase = Phase.COMPLETED;
                } else {
                    //currently within meeting room
                    //get out of meeting room
                }
            } else {
                //person did not accept invite
                //turn away from the person
            }
        }
    }

    private boolean isTaskComplete() {
        //extend to add 15min time limit and/or other limits
        return pplCount == targetPplCount;
    }

    private RectangleWithDepth findPerson() {
        //wait for some faces
        while (lastCameraData.length == 0) {
            //explore!!!!!!!
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                Logger.getLogger(MainNode.class.getName()).log(Level.SEVERE, null, ex);
            }
        }

        float[] cameraDataCopy = Arrays.copyOf(lastCameraData, lastCameraData.length);
        RectangleWithDepth[] rectangles = convert(cameraDataCopy);
        return getClosestRectangle(rectangles);
    }

    private RectangleWithDepth[] convert(float[] data) {
        int numberOfRectangles = (int) (data.length / 5.0);
        RectangleWithDepth[] result = new RectangleWithDepth[numberOfRectangles];
        for (int i = 0; i < numberOfRectangles; i++) {
            int index = numberOfRectangles * 5;
            result[i] = new RectangleWithDepth(
                    data[index],
                    data[index + 1],
                    data[index + 2],
                    data[index + 3],
                    data[index + 4]);
        }
        return result;
    }

    private RectangleWithDepth getClosestRectangle(RectangleWithDepth[] rectangles) {
        RectangleWithDepth closestRect = null;
        for (RectangleWithDepth rect : rectangles) {
            if (closestRect == null || rect.getDepth() < closestRect.getDepth()) {
                closestRect = rect;
            }
        }
        return closestRect;
    }

    private PoseStamped getPersonLocation(Pose estimatedPoseCopy, RectangleWithDepth areaOfPerson) {
        PoseStamped pose = messageFactory.newFromType(PoseStamped._TYPE);
        double heading = StaticMethods.getHeading(estimatedPoseCopy.getOrientation());
        double distance = areaOfPerson.getDepth() * 0.9;
        pose.getPose().getPosition().setX(
                estimatedPoseCopy.getPosition().getX()
                + (distance * Math.cos(heading)));
        pose.getPose().getPosition().setY(
                estimatedPoseCopy.getPosition().getY()
                + (distance * Math.sin(heading)));
        return pose;
    }

    private boolean personLost() {
        return false;
    }

    private boolean personAcceptsInvite() {
        return false;
    }

    private void setPRMGoal(PoseStamped targetLocation) {
        super.goal.publish(targetLocation);
        // 0 = no path found
        // 1 = path to goal found
        // 2 = Goal reached
    }

    public void updateMeetingRoomLocation(Point location) {
        meetingRoomLocation.getPose().setPosition(location);
    }

    @Override
    public void onNewCameraRectanglePoints(float[] data) {
        lastCameraData = data;
    }

    @Override
    public void onNewEstimatedPose(Pose estimatedPose) {
        lastEstimatedPose = estimatedPose;
    }
}
