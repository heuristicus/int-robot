package ex4;

import ex3.PRM;
import util.PRMUtil;
import ex3.Vertex;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Twist;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import javax.sound.sampled.Clip;
import launcher.RunParams;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;
import std_msgs.Float32MultiArray;
import std_msgs.Int32;
import util.GeneralUtil;
import util.MeetingUtil;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

public class MainNode extends AbstractNodeMain {

    private enum Phase {

        INITIALISATION,
        FINDROOM,
        SCANNINGROOM,
        FACECHECKINROOM,
        EXPLORING,
        FACECHECK,
        ROTATETOPERSON,
        PRMTOPERSON,
        PRMTOROOM,
        INMEETINGROOM,
        COMPLETED
    }
    public static Double INITIAL_EXPLORATION_GRID_STEP = RunParams.getDouble("INITIAL_EXPLORATION_GRID_STEP");
    public static Double MINIMUM_EXPLORATION_GRID_STEP = RunParams.getDouble("MINIMUM_EXPLORATION_GRID_STEP");
    public static Double GRID_SIZE_REDUCTION_STEP = RunParams.getDouble("GRID_SIZE_REDUCTION_STEP");
    public static Integer FACE_CONFIRM_DETECTIONS = RunParams.getInt("FACE_CONFIRM_DETECTIONS");
    public static Double MAX_RECTANGLE_CENTRE_DISPARITY = RunParams.getDouble("MAX_RECTANGLE_CENTRE_DISPARITY");
    public static Double MAX_RECTANGLE_DEPTH_DISPARITY = RunParams.getDouble("MAX_RECTANGLE_DEPTH_DISPARITY");
    public static Double FACE_CENTRED_THRESHOLD = RunParams.getDouble("FACE_CENTRED_THRESHOLD");
    public static Double INITIAL_EXPLORATION_CELL_SIZE = RunParams.getDouble("INITIAL_EXPLORATION_CELL_SIZE");
    public static Double MINIMUM_EXPLORATION_CELL_SIZE = RunParams.getDouble("MINIMUM_EXPLORATION_CELL_SIZE");
    public static Double CELL_SIZE_REDUCTION_STEP = RunParams.getDouble("CELL_SIZE_REDUCTION_STEP");
    public static Integer EXPLORATION_TARGET_PER_CELL = RunParams.getInt("EXPLORATION_TARGET_PER_CELL");
    public static String EXPLORATION_SAMPLING = RunParams.get("EXPLORATION_SAMPLING");
    public static String SUCCESS_SOUND_FILE = "/usr/lib/openoffice/basis3.2/share/gallery/sounds/applause.wav";
    public static Double FOV_DISTANCE = RunParams.getDouble("FOV_DISTANCE");
    public static Integer FOV_ANGLE = RunParams.getInt("FOV_ANGLE");
    public static Double FOV_MIN_DIST = RunParams.getDouble("FOV_MIN_DIST");
    public static Double FOV_ANGLE_STEP = RunParams.getDouble("FOV_ANGLE_STEP");
    private float[] lastCameraData;
    private Pose lastEstimatedPose;
    public double currentGridStep = INITIAL_EXPLORATION_GRID_STEP;
    public double currentCellSize = INITIAL_EXPLORATION_CELL_SIZE;
    private Phase currentPhase = Phase.INITIALISATION;
    private int pplCount = 0;
    private int targetPplCount = 2;
    public ArrayList<Vertex> explorationVertices;
    public int faceCheckCount;
    RectangleWithDepth lastFaceRectangle;
    public static MessageFactory messageFactory;
    public static Dimension CAMERA_DIMENSIONS = new Dimension(640, 480);
    protected Driver driver;
    public PRMUtil prmUtil;
    public GeneralUtil genUtil;
    public OccupancyGrid inflatedMap;
    public OccupancyGrid originalMap;
    private Polygon2D[] meetingRooms;
    private PoseStamped[] centreOfMeetingRooms;
    private int meetingRoomIndex = -1;
    private double turnRemaining = 0;
    Publisher<Twist> twist_pub;
    Publisher<PoseStamped> goal;
    Subscriber<Odometry> odom;
    Subscriber<std_msgs.Int32> prmInfo;
    Subscriber<std_msgs.Float32MultiArray> cameraRectSubscriber;
    Subscriber<PoseWithCovarianceStamped> estimatedPose;
    Subscriber<OccupancyGrid> inflatedMapSub;
    Subscriber<OccupancyGrid> originalMapSub;
    Publisher<std_msgs.Bool> navActivePub;
    Publisher<MarkerArray> explorationMarkerPub;
    public OccupancyGrid exploredMap; // Map to chart explored portions of the map
    private OccupancyGrid heatMap;
    private double[] heatMapData;
    Publisher<OccupancyGrid> heatMapPub;
    Publisher<OccupancyGrid> exploredMapPub;
    private static long explorationStartTime = 0;
    int a = 1; // DEBUGGING. PLEASE DELETE

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
        explorationMarkerPub.setLatchMode(true);
        exploredMapPub = connectedNode.newPublisher("exp_map", OccupancyGrid._TYPE);
        exploredMapPub.setLatchMode(true);

        heatMapPub = connectedNode.newPublisher("heat_map", OccupancyGrid._TYPE);
        heatMapPub.setLatchMode(true);

        //instantiate the driver with the twist publisher
        driver = new Driver(twist_pub);

        // This section is initialising the position of the rooms and the rectangles of the room 
        // We are also hard coding the centre of the rooms
        meetingRooms = new Polygon2D[2];
        meetingRooms[0] = new Polygon2D(new double[]{16.115, 20.435, 18.427, 13.835}, new double[]{15.175, 10.499, 8.38, 12.992}, 4);
        meetingRooms[1] = new Polygon2D(new double[]{11.993, 15.885, 13.793, 9.703}, new double[]{19.136, 15.258, 12.987, 17.096}, 4);

        PoseStamped centreMeetingRoomOne = messageFactory.newFromType(PoseStamped._TYPE);
        PoseStamped centreMeetingRoomTwo = messageFactory.newFromType(PoseStamped._TYPE);
        centreMeetingRoomOne.getPose().getPosition().setX(17.1);
        centreMeetingRoomOne.getPose().getPosition().setY(11.75);
        centreMeetingRoomTwo.getPose().getPosition().setX(12.4);
        centreMeetingRoomTwo.getPose().getPosition().setY(16.6);

        centreOfMeetingRooms = new PoseStamped[2];
        centreOfMeetingRooms[0] = centreMeetingRoomOne;
        centreOfMeetingRooms[1] = centreMeetingRoomTwo;

        odom = connectedNode.newSubscriber("odom", Odometry._TYPE);
        odom.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry t) {
                driver.onNewOdomMessage(t);

                if (currentPhase == Phase.SCANNINGROOM) {
                    if (driver.isTargetReached()) {
                        Printer.println("Driver rotation target reached", "REDB");
                        Printer.println("Room is free. Exploring for humans!");
                        initialiseExploration();
                    }
                }

//                if (lastEstimatedPose == null){
//                    return;
//                }
//                double currentHeading = getHeadingFromLastPos();
//                Printer.println("Current heading is " + currentHeading, "REDF");
//                if (a == 1 || driver.isTargetReached()) {
//                    a = 2;
//                    Printer.println("Initialisation rotation test...", "REDF");
//                    int angle = -360;
//                    currentHeading = getHeadingFromLastPos();
//                    driver.turn(currentHeading, Math.toRadians(angle));
//                }

//                if (currentPhase == Phase.SCANNINGROOM) {
//                    if (turnRemaining >= 0) {
//                        Printer.println("Got initial pose... Initialising exploratio", "CYANF");
//
//                    }
//                }
            }
        });

        prmInfo = connectedNode.newSubscriber("goalInfo", std_msgs.Int32._TYPE);
        prmInfo.addMessageListener(new MessageListener<Int32>() {

            @Override
            public void onNewMessage(Int32 t) {
                if (t.getData() == PRM.GOAL_REACHED) {
                    if (currentPhase == Phase.FINDROOM) {
                        //start scanning the room (360);
                        scanRoomForFace();
                    } else if (currentPhase == Phase.EXPLORING) {
                        if (explorationVertices.size() > 0) {
                            goToNextExplorationVertex();
                        } else {
                            // Exploration path done. Let's go again, but increase
                            // the granularity
//                            System.exit(0);
                            exploreWithMoreGranularity();
                        }
                    } else if (currentPhase == Phase.PRMTOPERSON) {
                        //Ask person if they want to go to meeting room, if yes prm to room else if no turn and continue exploring
                        DialogBox dialog = new DialogBox("Attend Meeting?", "Would you like to attend the meeting?");
                        if (dialog.getUserResponse() == DialogBox.response.YES) {
                            Printer.println("Person accepted invite, PRMing to meeting room", "CYANF");
                            prmToMeetingRoom();
                        } else {
                            Printer.println("Person rejected or timed out invite, continuing exploration", "CYANF");
                            goToNextExplorationVertex();
                            returnToExploration();
                        }
                    } else if (currentPhase == Phase.PRMTOROOM) {
                        // we are at the room now, drop person off and tell them they are at meeting room and then begin exploring again
                        Printer.println("Currently in a room, returning to exploration", "CYANF");
                        pplCount++;
                        // Before leaving meeting room check if the task is complete, if so then print statement else continue
                        if (isTaskComplete()) {
                            Printer.println("I have completed the whole task", "GREENB");
                            DialogBox.playAlertSound(DialogBox.SUCCESS_SOUND);
                            currentPhase = Phase.COMPLETED;
                        } else {
                            goToNextExplorationVertex();
                            returnToExploration();
                        }
                    }
                } else if (t.getData() == PRM.NO_PATH) {
                    // If we can't find a route to the exploration node, give up
                    // and go to the next one.
                    if (currentPhase == Phase.EXPLORING) {
                        Printer.println("PRM could not find path. Popping next vertex.", "REDF");
                    } else if (currentPhase == Phase.PRMTOPERSON) {
                        Printer.println("Person location blocked by obstacles. Continuing exploration.", "REDF");
                    }
                    goToNextExplorationVertex();
                    returnToExploration();
                } else if (t.getData() == PRM.PATH_FOUND) {
                    System.out.println("Path found.");
                }
            }
        });

        inflatedMapSub = connectedNode.newSubscriber("inflatedMap", OccupancyGrid._TYPE);
        inflatedMapSub.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid t) {
                if (currentPhase == Phase.INITIALISATION) {
                    Printer.println("Got map in MainNode", "CYANF");
                    inflatedMap = t;
                    prmUtil = new PRMUtil(new Random(), messageFactory, inflatedMap);
                    genUtil = new GeneralUtil(messageFactory);
                    if (exploredMap == null) {
                        exploredMap = GeneralUtil.copyMap(originalMap, messageFactory);
                    }
                }
            }
        });

        originalMapSub = connectedNode.newSubscriber("map", OccupancyGrid._TYPE);
        originalMapSub.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid t) {
                if (originalMap == null) {
                    originalMap = t;
                    heatMap = messageFactory.newFromType(OccupancyGrid._TYPE);
                    heatMap.setHeader(originalMap.getHeader());
                    heatMap.setInfo(originalMap.getInfo());
                    heatMap.setData(originalMap.getData().copy());
                    heatMap.getData().clear();
                    heatMapData = new double[heatMap.getData().array().length];
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
                lastCameraData = t.getData();
                if (t.getData().length != 0) {
                    if (currentPhase == Phase.SCANNINGROOM) {
                        Printer.println("Scanning room found face. Pausing and investigating face", "REDF");
                        driver.pauseTurning();
                        currentPhase = Phase.FACECHECKINROOM;
                    }

                    if (currentPhase == Phase.EXPLORING) {
                        std_msgs.Bool deactivate = navActivePub.newMessage();
                        deactivate.setData(false);
                        navActivePub.publish(deactivate);
                        currentPhase = Phase.FACECHECK;
                        Printer.println("Face seen. Stopping and investigating face", "CYANF");
                    }
                }

                if (currentPhase == Phase.FACECHECK || currentPhase == Phase.FACECHECKINROOM) { // if we are checking faces
                    // if we have not received enough messages to confirm a face
                    if (faceCheckCount < FACE_CONFIRM_DETECTIONS) {
                        if (t.getData().length == 0) {
                            // If we receive a zero length array while we are
                            // attempting to confirm a face, we abandon the check.

                            if (currentPhase == Phase.FACECHECK) {
                                Printer.println("Lost face (no faces). Returning to exploration", "CYANF");
                                returnToExploration();
                            } else if (currentPhase == Phase.FACECHECKINROOM) {
                                Printer.println("Lost face (no faces). Continuing to scan room", "CYANF");
                                returnToScanningRoom();
                            }
                        }
                        RectangleWithDepth newFaceRect = findPerson(lastFaceRectangle);
                        // Check whether the rectangle received is close to the
                        // one we received in the previous message.
                        if (newFaceRect != null && (lastFaceRectangle == null ||
                                MeetingUtil.rectangleOverlapValid(lastFaceRectangle, newFaceRect, MAX_RECTANGLE_CENTRE_DISPARITY))) {
                            faceCheckCount++;
                            lastFaceRectangle = newFaceRect;
                            Printer.println("Face matches last seen. FaceCheckCount=" + faceCheckCount, "CYANF");
                        } else {
                            // If the rectangles are too dissimilar, we return to
                            // the exploration phase
                            Printer.println("Lost face (dissimilar). Returning to exploration", "CYANF");
                            if (currentPhase == Phase.FACECHECKINROOM) {
                                returnToScanningRoom();
                            } else if (currentPhase == Phase.FACECHECK) {
                                returnToExploration();
                            }

                            return;
                        }
                    }

                    // If we have made enough detections to confirm a face,
                    // set phase to rotate to person
                    if (faceCheckCount == FACE_CONFIRM_DETECTIONS) {
                        faceCheckCount = 0;
                        if (currentPhase == Phase.FACECHECKINROOM) {
                            if (meetingRoomIndex < meetingRooms.length) {
                                findEmptyRoom();
                                Printer.println("Face confirmed, therefore meeting room is not empty. Finding new meeting room.", "CYANF");
                            } else {
                                Printer.println("Face confirmed, therefore meeting room is not empty. No more rooms remaining.", "REDB");
                            }
                        } else if (currentPhase == Phase.FACECHECK) {
                            currentPhase = Phase.ROTATETOPERSON;
                            rotateTowardsPerson(findPerson(lastFaceRectangle));
                            Printer.println("Face confirmed. Rotating to person", "CYANF");
                        }
                    }
                }

                if (currentPhase == Phase.ROTATETOPERSON) {
                    if (t.getData().length == 0) {
                        Printer.println("Person lost while rotating - returning to exploration.", "CYANF");
                        driver.stopTurning();
                        returnToExploration();
                        return;
                    }

                    // If we've not yet reached the target angle, then return.
                    // note that the rotation is done in small increments, so
                    // the initial target is not necessarily the full rotation to
                    // the heading which faces the person.
                    if (!driver.isTargetReached()) {
                        return;
                    }
                    if (isFaceCentred(lastFaceRectangle)) {
                        Printer.println("Face in centre. PRMing to person", "CYANF");
                        currentPhase = Phase.PRMTOPERSON;
                        setPRMGoal(MeetingUtil.getObjectLocation(lastEstimatedPose, lastFaceRectangle.depth, messageFactory));
                        Printer.println("Person goal set. Heading over...", "CYANF");
                    } else {
                        RectangleWithDepth rect = findPerson(lastFaceRectangle);
                        if (rect != null) {
                            Printer.println("Face rectangle received was null. Returning to exploration.", "CYANF");
                            returnToExploration();
                            return;
                        } else {
                            Printer.println("Face not in centre. Rotating towards person again", "CYANF");
                            rotateTowardsPerson(rect);
                        }
                    }
                }
            }
        });

        //set the subscriber for the estimated pose
        estimatedPose = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
        estimatedPose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {

            @Override
            public void onNewMessage(PoseWithCovarianceStamped message) {
                lastEstimatedPose = StaticMethods.copyPose(message.getPose().getPose());

                if (currentPhase == Phase.INITIALISATION && inflatedMap != null) {
                    //driver.onNewEstimatedPose(lastEstimatedPose);
                    //findEmptyRoom();
                    initialiseExploration();
                }
            }
        });
        Printer.println("MainNode initialised", "CYANF");

        //set the subscriber for the ground truth position (from stage)
        Subscriber<Odometry> groundTruth = connectedNode.newSubscriber("base_pose_ground_truth", Odometry._TYPE);
        groundTruth.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry message) {
                 Pose lastRealPos = StaticMethods.copyPose(message.getPose().getPose());
                // Invert x and y so that they match the actual position.
                // Why? I have no idea.
//                double y_temp = lastRealPos.getPosition().getY();// * -1.0;
//                lastRealPos.getPosition().setY(lastRealPos.getPosition().getX());
//                lastRealPos.getPosition().setX(y_temp);

                if (exploredMap != null && explorationVertices != null) {
                    Time time = message.getHeader().getStamp();
//                   plotFieldOfViewOnMap(exploredMap, lastRealPos, time);
                    // Appears to require subtraction 0.6 to get the stage base pose to correspond
                    // with correct position in rviz
//                    ArrayList<ArrayList<Integer>> rayIndexes = MeetingUtil.projectFOV(-lastRealPos.getPosition().getY() - 0.6,
//                            lastRealPos.getPosition().getX() - 0.6,
//                            AbstractLocaliser.getHeading(lastRealPos.getOrientation()),
//                            FOV_ANGLE, FOV_MIN_DIST, FOV_DISTANCE, FOV_ANGLE_STEP,
//                            originalMap, exploredMap, meetingRooms, time);
                    ArrayList<Integer> rayIndexes = MeetingUtil.simple_projectFOV(-lastRealPos.getPosition().getY() - 0.6,
                            lastRealPos.getPosition().getX() - 0.6,
                            AbstractLocaliser.getHeading(lastRealPos.getOrientation()),
                            FOV_ANGLE, FOV_MIN_DIST, FOV_DISTANCE, FOV_ANGLE_STEP, 
                            originalMap, exploredMap, meetingRooms, time);
                    exploredMapPub.publish(exploredMap);
                    MeetingUtil.updateHeatData(rayIndexes, heatMapData);
                    MeetingUtil.normaliseHeatMap(heatMap, heatMapData);
                    heatMapPub.publish(heatMap);
                }
            }
        });
    }

    public void plotFieldOfViewOnMap(OccupancyGrid explorationMap, Pose pose, Time time) {
        final int mapHeight = explorationMap.getInfo().getHeight();
        final int mapWidth = explorationMap.getInfo().getWidth();
        final float mapRes = explorationMap.getInfo().getResolution();
        Polygon2D tri = MeetingUtil.getFOVpoly(pose, FOV_MIN_DIST, FOV_DISTANCE, FOV_ANGLE, mapRes);

        int index;
        int pixel;
        int freeBefore = 0;
        int freeAfter = 0;
        for (int y = 0; y < mapHeight; y++) {
            for (int x = 0; x < mapWidth; x++) {
                index = GeneralUtil.getMapIndex(y, x, mapWidth, mapHeight); // X and y flipped. Go figure
                pixel = explorationMap.getData().getByte(index);
                if (pixel != 100 && pixel != -1) {
                    if (!MeetingUtil.isPointInMeetingRooms(meetingRooms, y * mapRes, x * mapRes)) { // X and y flipped. Go figure
                        freeBefore++;
                        if (tri.contains(x, y)) {
                            explorationMap.getData().setByte(index, 100);
                        } else {
                            freeAfter++;
                        }
                    }
                }
                if (!MeetingUtil.isPointInMeetingRooms(meetingRooms, y * mapRes, x * mapRes) && tri.contains(x, y)) { // X and y flipped. Go figure
                    int mapCellData = originalMap.getData().getByte(index);
//                    Printer.println("Entering heat map code, Map cell value:" + mapCellData, "CYANB");
                    if (mapCellData >= 0 && mapCellData <= 65) { //not occupied
                        heatMapData[index]++;
                        //Printer.println("Heat Map Data i:" + index + " value: " + heatMapData[index], "REDB");
                    }
                }
            }
        }

        double orientation = Math.toDegrees(AbstractLocaliser.getHeading(pose.getOrientation())) + 90;
        if (orientation > 180) {
            orientation = -(180 - orientation % 180);
        }

        long now = time.secs;
        if (explorationStartTime == 0) {
            explorationStartTime = now;
        } //System.currentTimeMillis(); }
        Printer.println("coverage: FREE AFTER:" + freeAfter + "," + (now - explorationStartTime) +
                " POSE: " + -pose.getPosition().getY() + "," + pose.getPosition().getX() + "," + orientation);
    }

    public double getHeadingFromLastPos() {
        return AbstractLocaliser.getHeading(lastEstimatedPose.getOrientation());
    }

    public void exploreWithMoreGranularity() {
        if (EXPLORATION_SAMPLING.equals("cell")) {
            if (currentCellSize > MINIMUM_EXPLORATION_CELL_SIZE) {
                currentCellSize -= CELL_SIZE_REDUCTION_STEP;
                System.out.println("Cell size set to " + currentCellSize);
            } else {
                Printer.println("Minimum cell size reached.");
                System.exit(0);
            }
        } else if (EXPLORATION_SAMPLING.equals("grid")) {
            if (currentGridStep > MINIMUM_EXPLORATION_GRID_STEP) {
                currentGridStep -= GRID_SIZE_REDUCTION_STEP;
                System.out.println("Grid size set to " + currentGridStep);
            } else {
                Printer.println("Minimum grid size reached.");
                System.exit(0);
            }
        }
        initialiseExploration();
    }

    public void findEmptyRoom() {
        Printer.println("Kicking off find-room phase", "CYANF");
        currentPhase = Phase.FINDROOM;
        meetingRoomIndex++;
        if (PRMUtil.getEuclideanDistance(centreOfMeetingRooms[0].getPose().getPosition(), lastEstimatedPose.getPosition())
                > PRMUtil.getEuclideanDistance(centreOfMeetingRooms[1].getPose().getPosition(), lastEstimatedPose.getPosition())) {
            Printer.println("Room 1 was closer than room 0. Going to room 1.", "CYANF");
            PoseStamped tmp = centreOfMeetingRooms[0];
            centreOfMeetingRooms[0] = centreOfMeetingRooms[1];
            centreOfMeetingRooms[1] = tmp;
        }
        if (meetingRoomIndex >= centreOfMeetingRooms.length) {
            // No more free rooms! Panic
            Printer.println("NO FREE ROOMS! Exiting :(", "REDB");
            Clip clip = DialogBox.playAlertSound(DialogBox.FAILURE_SOUND);
            try {
                if (clip != null) {
                    while (!clip.isRunning()) {
                        Thread.sleep(10);
                    }
                    while (clip.isRunning()) {
                        Thread.sleep(10);
                    }
                    clip.close();
                }
            } catch (Exception e) {
            }

            meetingRoomIndex = 0; // Restart meeting room search
//            System.exit(0);
        }
        setPRMGoal(centreOfMeetingRooms[meetingRoomIndex]);
    }

    public void scanRoomForFace() {
        Printer.println("Scanning room for face", "CYANF");
        currentPhase = Phase.SCANNINGROOM;
        turnRemaining = Math.PI * 2;
        driver.turn(getHeadingFromLastPos(), turnRemaining);
    }

    public void returnToScanningRoom() {
        //driver.turn(turnRemaining, false, false);
        //driver.turn(getHeadingFromLastPos(), turnRemaining);
        currentPhase = Phase.SCANNINGROOM;
        driver.resumeTurning();
        lastFaceRectangle = null;
        faceCheckCount = 0;
    }

    /*
     * Returns the node state to exploration phase from the facecheck phase
     */
    public void returnToExploration() {
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
        ArrayList<Vertex> vertices = null;
        if (EXPLORATION_SAMPLING.equals("grid")) {
            vertices = prmUtil.gridSample(inflatedMap,
                    currentGridStep, currentGridStep);
        } else if (EXPLORATION_SAMPLING.equals("cell")) {
            vertices = prmUtil.cellSample(inflatedMap,
                    currentCellSize, EXPLORATION_TARGET_PER_CELL);
        }

        explorationVertices = MeetingUtil.getExplorationPath(lastEstimatedPose, vertices);
        MeetingUtil.removeVerticesInMeetingRooms(explorationVertices, meetingRooms);
        currentPhase = Phase.EXPLORING;

        MarkerArray markers = explorationMarkerPub.newMessage();
        Marker m = genUtil.makePathMarker(explorationVertices, "expvert", null, 23);
        ArrayList<Marker> mlist = new ArrayList<Marker>();
        mlist.add(m);
        markers.setMarkers(mlist);
        explorationMarkerPub.publish(markers);
        Printer.println("coverage: number of waypoints: " + explorationVertices.size() + ", values:" + explorationVertices);
        goToNextExplorationVertex();
        Printer.println("Exploration initialised and markers published", "CYANF");
    }

    /*
     * Send a goal message to the PRM containing the next position that the
     * explorer should go to.
     */
    public void goToNextExplorationVertex() {
        Printer.println("Going to next exploration vertex.", "REDF");
        if (explorationVertices.isEmpty()) {
            exploreWithMoreGranularity();
            return;
        }
        Vertex nextVertex = this.explorationVertices.remove(0);
        PoseStamped goalPose = messageFactory.newFromType(PoseStamped._TYPE);
        goalPose.getPose().setPosition(nextVertex.getLocation());
        setPRMGoal(goalPose);
    }


    /**
     * Change current phase to prm to meeting room
     * set the PRM goal to be the location of the empty meeting room
     */
    private void prmToMeetingRoom() {
        currentPhase = Phase.PRMTOROOM;
        setPRMGoal(centreOfMeetingRooms[meetingRoomIndex]);
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
        RectangleWithDepth[] rectangles = MeetingUtil.convert(cameraDataCopy);
        if (previousRect == null) {
            return MeetingUtil.getClosestRectangle(rectangles);
        } else {
            // find the rectangle that is the closest match to the one that we
            // received as a parameter
            return MeetingUtil.getMostSimilarRectangle(rectangles, previousRect, MAX_RECTANGLE_DEPTH_DISPARITY, messageFactory);
        }
    }

    /*
     * Checks whether the rectangle defining the face detection is withing some
     * distance of the centre of the image.
     */
    public boolean isFaceCentred(RectangleWithDepth personDetection) {
        return Math.abs(personDetection.getCenterX() - (CAMERA_DIMENSIONS.width / 2)) < FACE_CENTRED_THRESHOLD * CAMERA_DIMENSIONS.width;
    }

    /*
     * Rotates the robot towards the centre of a given rectangle. This rectangle
     * is assumed to be the location of a person.
     */
    public void rotateTowardsPerson(RectangleWithDepth lastRectangle) {
        double areaCentreX = lastRectangle.getCenterX();
        double distFromCentreX = areaCentreX - (CAMERA_DIMENSIONS.width / 2);
        Printer.println("DistFromCentreX: " + distFromCentreX, "CYANF");
        double turnAngle = Math.toRadians(5);
        if (distFromCentreX > 0) {
            //rectangle on the right
            //driver.turn(-turnAngle, true, false);
            driver.turn(getHeadingFromLastPos(), -turnAngle);
        } else {
            //rectangle on left
            //driver.turn(turnAngle, true, false);
            driver.turn(getHeadingFromLastPos(), turnAngle);
        }
    }

    /*
     * Publishes a goal to the PRM topic.
     */
    private void setPRMGoal(PoseStamped targetLocation) {
        goal.publish(targetLocation);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Arbitrator");
    }
}
