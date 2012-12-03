/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import ex3.PRM;
import ex3.PRMUtil;
import ex3.Vertex;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Arrays;
import launcher.RunParams;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Float32MultiArray;
import std_msgs.Int32;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

/**
 *
 * @author HammyG
 */
public class MainNode extends AbstractNodeMain {

     private enum Phase {
        INITIALISATION,
        FINDINGROOM,
        EXPLORING,
        FACECHECK,
        PRMTOPERSON,
        PRMTOROOM,
        INMEETINGROOM,
        COMPLETED
    }

    public static double INITIAL_EXPLORATION_GRID_STEP = RunParams.getDouble("INITIAL_EXPLORATION_GRID_STEP");
    public static double MINIMUM_EXPLORATION_GRID_STEP = RunParams.getDouble("MINIMUM_EXPLORATION_GRID_STEP");
    public static int FACE_CONFIRM_DETECTIONS = RunParams.getInt("FACE_CONFIRM_DETECTIONS");
    public static double MAX_RECTANGLE_CENTRE_DISPARITY = RunParams.getDouble("MAX_RECTANGLE_CENTRE_DISPARITY");
    public static double MAX_RECTANGLE_DEPTH_DISPARITY = RunParams.getDouble("MAX_RECTANGLE_DEPTH_DISPARITY");

    private float[] lastCameraData;
    private Pose lastEstimatedPose;
    private PoseStamped meetingRoomLocation = null;
    private Phase currentPhase = Phase.FINDINGROOM;
    private int pplCount = 0;
    private int targetPplCount = 3;
    public ArrayList<Vertex> explorationVertices;
    public int faceCheckCount;
    RectangleWithDepth lastFaceRectangle;

    public static MessageFactory messageFactory;
    public static Dimension CAMERA_DIMENSIONS = new Dimension(640, 480);
    protected Driver driver;
    public PRMUtil prmUtil;
    public OccupancyGrid map;

    Publisher<Twist> twist_pub;
    Publisher<PoseStamped> goal;
    Subscriber<Odometry> odom;
    Subscriber<std_msgs.Int32> prmInfo;
    Subscriber<std_msgs.Float32MultiArray> cameraRectSubscriber;
    Subscriber<PoseStamped> estimatedPose;
    Subscriber<OccupancyGrid> mapSub;
    Publisher<std_msgs.Bool> navActivePub;
    Publisher<MarkerArray> explorationMarkerPub;
    
    @Override
    public void onStart(ConnectedNode connectedNode) {
        //set up the message factory
        messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
        StaticMethods.messageFactory = messageFactory;


        //set publisher for moving the robot
        twist_pub = connectedNode.newPublisher("cmd_vel", Twist._TYPE);

        // Publisher for the goal which prm shall use
        goal = connectedNode.newPublisher("goal", PoseStamped._TYPE);

        // Publisher to activate or deactivate navigator.
        navActivePub = connectedNode.newPublisher("nav_active", std_msgs.Bool._TYPE);

        explorationMarkerPub = connectedNode.newPublisher("exp_vert", MarkerArray._TYPE);
        //instantiate the driver with the twist publisher
        driver = new Driver(twist_pub);

        odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
        odom.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry t) {
                driver.onNewOdomMessage(t);
            }
        });

        prmInfo = connectedNode.newSubscriber("goalInfo", std_msgs.Int32._TYPE);
        prmInfo.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 t) {
                if (t.getData() == PRM.GOAL_REACHED) {
                    if (currentPhase == Phase.EXPLORING){
                        goToNextExplorationVertex();
                    }
                } else if (t.getData() == PRM.NO_PATH) {
                    System.out.println("Could not find path");
                } else if (t.getData() == PRM.PATH_FOUND) {
                    System.out.println("Path found.");
                }
            }
        });

        mapSub = connectedNode.newSubscriber("inflatedMap", OccupancyGrid._TYPE);
        mapSub.addMessageListener(new MessageListener<OccupancyGrid>() {
            @Override
            public void onNewMessage(OccupancyGrid t) {
                if (currentPhase == Phase.INITIALISATION) {
                    map = t;
                }
            }
        });

        //set up subscriber for the rectangles from the opencv node. Messages
        // are published even if there are no faces detected, but they will be
        // empty (length 0)
        cameraRectSubscriber = connectedNode.newSubscriber("face_rects", std_msgs.Float32MultiArray._TYPE);
        cameraRectSubscriber.addMessageListener(new MessageListener<Float32MultiArray>() {
            @Override
            public void onNewMessage(Float32MultiArray t) {
                onNewCameraRectanglePoints(t.getData());
                if (currentPhase == Phase.EXPLORING && t.getData().length != 0) {
                    std_msgs.Bool deactivate = navActivePub.newMessage();
                    deactivate.setData(false);
                    navActivePub.publish(deactivate);
                    currentPhase = Phase.FACECHECK;
                }

                if (currentPhase == Phase.FACECHECK){ // if we are checking faces
                    // if we have not received enough messaged to confirm a face
                    if (faceCheckCount < FACE_CONFIRM_DETECTIONS) {
                        if (t.getData().length == 0){
                            // If we receive a zero length array while we are
                            // attempting to confirm a face, we abandon the check.
                            returnToExploration();
                        }
                        RectangleWithDepth newFaceRect = findPerson(lastFaceRectangle);
                        // Check whether the rectangle received is close to the
                        // one we received in the previous message.
                        if (rectangleOverlapValid(lastFaceRectangle, newFaceRect)){
                            faceCheckCount++;
                            lastFaceRectangle = newFaceRect;
                        } else {
                            // If the rectangles are too dissimilar, we return to
                            // the exploration phase
                            returnToExploration();
                        }
                    }

                    // If we have made enough detections to confirm a face,
                    // set a prm goal to get to the person.
                    if (faceCheckCount == FACE_CONFIRM_DETECTIONS) {
                        faceCheckCount = 0;
                        currentPhase = Phase.PRMTOPERSON;
                        setPRMGoal(getPersonLocation(lastEstimatedPose, lastFaceRectangle));
                        lastFaceRectangle = null;
                    }
                }
            }
        });

        //set the subscriber for the estimated pose
        estimatedPose = connectedNode.newSubscriber("estimated_pose", PoseStamped._TYPE);
        estimatedPose.addMessageListener(new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped message) {
                lastEstimatedPose = StaticMethods.copyPose(message.getPose());
                if (currentPhase == Phase.INITIALISATION && map != null){
                    initialiseExploration();
                }
            }
        });
    }

    /*
     * Returns the node state to exploration phase from the facecheck phase
     */
    public void returnToExploration(){
        std_msgs.Bool activate = navActivePub.newMessage();
        activate.setData(true);
        navActivePub.publish(activate);
        currentPhase = Phase.EXPLORING;
        lastFaceRectangle = null;
        faceCheckCount = 0;
    }

    /*
     * Called when we have received a map in order to initialise the structures
     * needed to perform exploration of the map.
     */
    public void initialiseExploration() {
        ArrayList<Vertex> gridVertices = prmUtil.gridSample(map,
                INITIAL_EXPLORATION_GRID_STEP, INITIAL_EXPLORATION_GRID_STEP);

        explorationVertices = getExplorationPath(lastEstimatedPose, gridVertices);
        currentPhase = Phase.EXPLORING;
        
        MarkerArray markers = explorationMarkerPub.newMessage();
        Marker m = prmUtil.makePathMarker(explorationVertices, "expvert", null, 23);
        ArrayList<Marker> mlist = new ArrayList<Marker>();
        mlist.add(m);
        markers.setMarkers(mlist);
        explorationMarkerPub.publish(markers);
    }

    public ArrayList<Vertex> getExplorationPath(Pose startPose, ArrayList<Vertex> vertices){
        ArrayList<Vertex> explorePath = new ArrayList<Vertex>(vertices.size());
        Point curPoint = startPose.getPosition();

        while (vertices.size() > 0) {
            int minIndex = -1;
            double minDist = Double.MAX_VALUE;
            for (int i = 0; i < vertices.size(); i++) {
                double thisDist = PRMUtil.getEuclideanDistance(curPoint, vertices.get(i).getLocation());
                if (thisDist < minDist){
                    minIndex = i;
                    minDist = thisDist;
                }
            }
            Vertex curVertex = vertices.remove(minIndex);
            curPoint = curVertex.getLocation();
            explorePath.add(curVertex);
        }
        return explorePath;
    }

    /*
     * Send a goal message to the PRM containing the next position that the
     * explorer should go to.
     */
    public void goToNextExplorationVertex(){
        Vertex nextVertex = this.explorationVertices.remove(0);
        PoseStamped goalPose = messageFactory.newFromType(PoseStamped._TYPE);
        goalPose.getPose().setPosition(nextVertex.getLocation());
        setPRMGoal(goalPose);
    }

    public void start() {
        meetingRoomLocation = messageFactory.newFromType(PoseStamped._TYPE);
        while (meetingRoomLocation == null) {
            //find room
        }
        while (currentPhase != Phase.COMPLETED) {
            currentPhase = Phase.EXPLORING;
            RectangleWithDepth areaOfPerson = findPerson(null);
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
                areaOfPerson = findPerson(null);
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

    /*
     * Find the rectangle in the array closest to the one that we received
     * previously. Otherwise, we select the closest rectangle to track.
     */
    private RectangleWithDepth findPerson(RectangleWithDepth previousRect) {
        float[] cameraDataCopy = Arrays.copyOf(lastCameraData, lastCameraData.length);
        RectangleWithDepth[] rectangles = convert(cameraDataCopy);
        if (previousRect == null) {
            return getClosestRectangle(rectangles);
        } else {
            // find the rectangle that is the closest match to the one that we
            // received as a parameter
            return getMostSimilarRectangle(rectangles, previousRect);
        }
    }

    /*
     * Checks the disparity between two rectangles is below the value specified
     * in the parameter file.
     */
    public boolean rectangleOverlapValid(RectangleWithDepth lastRect, RectangleWithDepth curRect){
        Point lastCentre = getRectCentre(lastRect);
        Point curCentre = getRectCentre(curRect);

        double xDisparity = Math.abs(curCentre.getX() - lastCentre.getX());
        double yDisparity = Math.abs(curCentre.getY() - lastCentre.getY());

        boolean xValid = xDisparity <= (MAX_RECTANGLE_CENTRE_DISPARITY * curRect.width);
        boolean yValid = yDisparity <= (MAX_RECTANGLE_CENTRE_DISPARITY * curRect.height);

        return xValid && yValid;
    }

    public Point getRectCentre(RectangleWithDepth rect){
        Point centre = messageFactory.newFromType(Point._TYPE);
        double lastX = rect.x + rect.width / 2;
        double lastY = rect.y + rect.height / 2;
        centre.setX(lastX);
        centre.setY(lastY);
        return centre;
    }

    /*
     * Checks whether the centre point of curRect is within the rectangle lastRect.
     */
    public boolean checkPointCentreInRectangle(RectangleWithDepth lastRect, RectangleWithDepth curRect){
        Point centre = getRectCentre(curRect);

        boolean xInRect = centre.getX() > lastRect.x && centre.getX() < lastRect.x + lastRect.width;
        boolean yInRect = centre.getY() > lastRect.y && centre.getY() < lastRect.y + lastRect.height;

        return xInRect && yInRect;
    }

    public void turnToPerson() {
        currentPhase = Phase.EXPLORING;
        RectangleWithDepth areaOfPerson = findPerson(null);
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
            areaOfPerson = findPerson(null);
            areaCenterX = areaOfPerson.getCenterX();
            fromCenterX = areaCenterX - (CAMERA_DIMENSIONS.width / 2);
        }
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

    /*
     * Find the rectangle in the array that has the smallest depth.
     */
    private RectangleWithDepth getClosestRectangle(RectangleWithDepth[] rectangles) {
        RectangleWithDepth closestRect = null;
        for (RectangleWithDepth rect : rectangles) {
            if (closestRect == null || rect.getDepth() < closestRect.getDepth()) {
                closestRect = rect;
            }
        }
        return closestRect;
    }

    /*
     * Finds the rectangle in the array that is most similar to the rectangle
     * given.
     */
    private RectangleWithDepth getMostSimilarRectangle(RectangleWithDepth[] rectangles,
            RectangleWithDepth testRect){
        RectangleWithDepth mostSimilar = null;
        double similarDist = Double.MAX_VALUE;
        for (RectangleWithDepth rect : rectangles) {
            Point thisCentre = getRectCentre(rect);
            Point testCentre = getRectCentre(testRect);

            double msgDepth = rect.depth;
            double testDepth = testRect.depth;
            double thisDist = PRMUtil.getEuclideanDistance(testCentre, thisCentre);

            if (thisDist < similarDist && Math.abs(msgDepth - testDepth) <= MAX_RECTANGLE_DEPTH_DISPARITY) {
                mostSimilar = rect;
                similarDist = thisDist;
            }
        }
        return mostSimilar;
    }

    private PoseStamped getPersonLocation(Pose estimatedPoseCopy, RectangleWithDepth areaOfPerson) {
        PoseStamped personLocation = messageFactory.newFromType(PoseStamped._TYPE);
        double heading = StaticMethods.getHeading(estimatedPoseCopy.getOrientation());
        double distance = areaOfPerson.getDepth();
        personLocation.getPose().getPosition().setX(
                estimatedPoseCopy.getPosition().getX()
                + (distance * Math.cos(heading)));
        personLocation.getPose().getPosition().setY(
                estimatedPoseCopy.getPosition().getY()
                + (distance * Math.sin(heading)));
        return personLocation;
    }

    private boolean personLost() {
        return false;
    }

    private boolean personAcceptsInvite() {
        return false;
    }

    private void setPRMGoal(PoseStamped targetLocation) {
        goal.publish(targetLocation);
    }

    public void updateMeetingRoomLocation(Point location) {
        meetingRoomLocation.getPose().setPosition(location);
    }

    public void onNewCameraRectanglePoints(float[] data) {
        lastCameraData = data;
    }

    public void onNewEstimatedPose(Pose estimatedPose) {
        lastEstimatedPose = estimatedPose;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Arbitrator");
    }


}
