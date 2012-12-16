package ex3.experiment;

import ex3.PRM;
import util.PRMUtil;
import ex3.Vertex;
import ex3.search.Dijkstra;
import geometry_msgs.Point;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import launcher.RunParams;
import logging.Logger;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;
import std_msgs.Int32;

public class BatchExperimenter {

    private static final int EXPERIMENT_TIMEOUT_MS = RunParams.getInt("EXPERIMENT_TIMEOUT_MS");

    private static Publisher<PoseWithCovarianceStamped> initialPosePub;
    private static Publisher<PoseStamped> goalPub;

    private static PRM prm;
    private static ConnectedNode node;
    private static int expNum;
    /** Maps minor expNums to lists of values that will vary for each minor exp */
    private static List<List<String>> expValues;
    private static int[] expMinorNums;
    private static String key;
    private static Timer t;
    private static TimerTask timeout;
    private static Logger logger;
    private static boolean cancelled = false;

    public static void runAllExperiments(ConnectedNode connectedNode) 
            throws FileNotFoundException, IOException {
        System.out.println("RunAllExperiments called");
        node = connectedNode;
        logger = new Logger("BatchExperiments", true);
        System.out.println("Logger initialised");

        initialPosePub = node.newPublisher("initialpose", PoseWithCovarianceStamped._TYPE);
//        initialPosePub.setLatchMode(true);
        goalPub = node.newPublisher("goal", PoseStamped._TYPE);
//        goalPub.setLatchMode(true);

//        Subscriber<MarkerArray> markerSub = connectedNode.newSubscriber("pathMarkers", MarkerArray._TYPE);
//        markerSub.addMessageListener(new MessageListener<MarkerArray>() {
//            @Override
//            public void onNewMessage(MarkerArray t) {
//                System.out.println("Path markers message received. Processing...");
//                experimentDone(true);
//            }
//        });
//
//        Subscriber<PoseArray> routeSub = connectedNode.newSubscriber("route", PoseArray._TYPE);
//        routeSub.addMessageListener(new MessageListener<PoseArray>() {
//            @Override
//            public void onNewMessage(PoseArray t) {
//                System.out.println("Path received from PRM. Processing...");
//                experimentDone(true);
//            }
//        });

        Subscriber<std_msgs.Int32> prmInfo = connectedNode.newSubscriber("goalInfo", std_msgs.Int32._TYPE);
        prmInfo.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 t) {
                if (t.getData() == PRM.NO_PATH) {
                    System.out.println("PRM could not find a path.");
                    experimentDone(true);
                } else if (t.getData() == PRM.PATH_FOUND){
                    System.out.println("PRM found path.");
                    experimentDone(true);
                } else {
                    System.out.println("Received " + t.getData() + " from goalInfo topic, but don't know what to do with it.");
                }

            }
        });

        expNum = 1;
        boolean experimentsToDo = initialiseExperiments(expNum);
        if (experimentsToDo) {
            runNextExperiment();
        }
    }

    /** Sets up values in the expValues field. Returns whether there are 
     * more minor experiments to do. I.e. if it returns false, don't call
     * runNextExperiment() */
    public static boolean initialiseExperiments(int expNumber) {
        int minorNum = 1;

        expValues = new ArrayList<List<String>>();
        while (true) {
            key = RunParams.get("EXPERIMENT_" + expNumber + "_KEY_" + minorNum);
            if (key == null) {
                if (minorNum == 1) {
                    return false;
                }
                break;
            }
            String[] valuesArray = RunParams.get("EXPERIMENT_" + expNumber + "_VALUES_" + minorNum).split(",");
            List valuesList = Arrays.asList(valuesArray);
            expValues.add(valuesList);
            minorNum++;
        }
        expMinorNums = new int[minorNum - 1];
        Arrays.fill(expMinorNums, 0);
        System.out.println("VauesArray populated for expNum: "+expNumber);
        logger.logLine("Experiment " + expNumber + ":");
        return true;
    }

    public static void runNextExperiment() {
        // Loop over keys which have a range of values and load in the
        // current value for each key, before running the experiment
        for (int minor = 1; minor < expMinorNums.length+1; minor++) {
            System.out.println("RunNext: EXPERIMENT_" + expNum + "_KEY_" + minor);
            key = RunParams.get("EXPERIMENT_" + expNum + "_KEY_" + minor);
            List<String> listForKey = expValues.get(minor-1);
            String currentValueForKey = listForKey.get(expMinorNums[minor-1]);

            System.out.println("Overriding property " + key + "=" + currentValueForKey);
            System.out.println("Value before override: " + RunParams.get(key));
            RunParams.overrideProperty(key, currentValueForKey);
            System.out.println("Value after override: " + RunParams.get(key));
        }
        runExperimentForValues();
    }

    /** The MinorNums array contains values for which minor experiment
     * we are currently performing. We want to increment the end value,
     * but if that has reached the maximum value, set it to 0 and go back
     * one index in the array. */
    public static boolean incrementMinorNums(int index) {
        if (expMinorNums[index] == expValues.get(index).size()-1) {
            if (index != 0) {
                expMinorNums[index] = 0;
                return incrementMinorNums(index - 1);
            } else {
                return false;
            }
        } else {
            expMinorNums[index]++;
            return true;
        }
    }

    // Create a default configuration for the nodes that we will be creating
    private static NodeConfiguration conf = NodeConfiguration.newPrivate();
    // Create an executor to use to execute nodes.
    private static NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();

    public static void runExperimentForValues() {
        System.out.println("Creating new prm");
        prm = new PRM(new Dijkstra(), true);
        System.out.println("Executing prm. Started at: "+new Date().toString());
        cancelled = false;
        t = new Timer();
        int timeoutDuration = EXPERIMENT_TIMEOUT_MS;
        timeout = new TimerTask() {
            @Override
            public void run() {
                System.out.println("--------------- Experiment timed out. Cancelling...");
                experimentDone(false); // shuts down node and moves on
            }
        };
        t.schedule(timeout, timeoutDuration);
        exec.execute(prm, conf);

        try {
            System.out.println("Waiting for PRM to initialise");
            while(!prm.graphGenerationComplete() && ! cancelled){
                System.out.print(".");
                Thread.sleep(200);
            }
            System.out.println("");
        } catch (InterruptedException e) {
        }

        System.out.println("Constructing initial pose message");
        PoseWithCovarianceStamped initialPose = initialPosePub.newMessage();
        initialPose = getStartPose(initialPose, node.getTopicMessageFactory(), expNum);
        initialPosePub.publish(initialPose);
        System.out.println("Sent initial pose message");

        try {
            System.out.println("Waiting for pose message receipt");
            while (prm.getCurrentPosition() == null && ! cancelled){
                System.out.print(".");
                Thread.sleep(200);
            }
            System.out.println("");
        } catch (InterruptedException e) {
        }

        System.out.println("Constructing goal pose message");
        PoseStamped goal = goalPub.newMessage();
        goal = getGoalPose(goal, node.getTopicMessageFactory(), expNum);
        goalPub.publish(goal);
        System.out.println("Sent goal message");
    }

    /** Called when an experiment is over, either due to completion or 
     * timing out. The boolean parameter specifies which one, allowing this 
     * method to act accordingly. */
    public synchronized static void experimentDone(boolean completed) {
        t.cancel();
        cancelled = true;
        if (completed) {
            processPath();
        } else {
            moveToNextExperiment();
        }
    }

    /*
     * Processes a path from the PRM and logs relevant data.
     */
    public static void processPath(){
        ArrayList<Vertex> routeToGoal = prm.getRoute();
        ArrayList<Vertex> flatRouteToGoal = prm.getFlatRoute();

        if (routeToGoal == null){
            System.out.println("PRM could not find a route to the goal point.");
            logger.logLine("RUNPARAMS:" + RunParams.getAllPropertiesString());
            logger.logLine("RESULT WAS NULL!");
        } else {
            double length = PRMUtil.getPathLength(routeToGoal);
            double flatLength = PRMUtil.getPathLength(flatRouteToGoal);

            System.out.println("Experiment: Path of length " + length + " found and recorded.");
            System.out.println("Moreover, regeneration attempts: " + prm.getRegenerationsForLastSearch());
            logger.logLine("RUNPARAMS:" + RunParams.getAllPropertiesString());
            logger.logLine("PathOfLength:" + length);
            logger.logLine("PointsInPath:" + routeToGoal.size());
            logger.logLine("FlatPathOfLength:" + flatLength);
            logger.logLine("PointsInFlattenedPath:" + flatRouteToGoal.size());
            logger.logLine("RegenerationAttempts:" + prm.getRegenerationsForLastSearch());
        }
        moveToNextExperiment();
    }

    /*
     * Moves the experimenter on to the next experiment. This increments the
     * experiment number and initialises required values. Exits if there are no
     * more experiments left.
     */
    public static void moveToNextExperiment() {
        exec.shutdownNodeMain(prm);
        prm = null; // Allow garbage collection
        boolean moreMinorExperiments = incrementMinorNums(expMinorNums.length-1);
        if (! moreMinorExperiments) {
            expNum++;
            System.out.println("No more minor experiments. Setting expNum to: "+expNum);
            boolean experimentsToDo = initialiseExperiments(expNum);
            if (! experimentsToDo) {
                System.out.println("No more experiments to do. Goodbye cruel world.");
                logger.close();
                System.exit(0);
            }
        }

        runNextExperiment();
    }

    /*
     * Gets the start pose that should be used in this experiment from the parameter file.
     */
    public static PoseWithCovarianceStamped getStartPose(PoseWithCovarianceStamped pose, MessageFactory factory, int expNum) {
        AbstractLocaliser.setFactory(factory);
        pose.getPose().getPose().setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(),
                RunParams.getDouble("EXPERIMENT_"+expNum+"_START_THETA"))); //approximate
        Point pt = factory.newFromType(Point._TYPE);
        double x = RunParams.getDouble("EXPERIMENT_"+expNum+"_STARTX");
        double y = RunParams.getDouble("EXPERIMENT_"+expNum+"_STARTY");
        pt.setX(x);
        pt.setY(y);
        pose.getPose().getPose().setPosition(pt);
        System.out.println("Start Heading: "+AbstractLocaliser.getHeading(pose.getPose().getPose().getOrientation())
                + "   x " + pose.getPose().getPose().getPosition().getX()
                + "   y " + pose.getPose().getPose().getPosition().getY());
        return pose;
    }

    /*
     * Gets the goal pose to be used with this experiment from the parameter file.
     */
    public static PoseStamped getGoalPose(PoseStamped pose, MessageFactory factory, int expNum) {
        pose.getPose().setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 
                RunParams.getDouble("EXPERIMENT_"+expNum+"_GOAL_THETA"))); //approximate
        Point pt = factory.newFromType(Point._TYPE);
        double x = RunParams.getDouble("EXPERIMENT_" + expNum + "_GOALX");
        double y = RunParams.getDouble("EXPERIMENT_"+expNum+"_GOALY");
        pt.setX(x);
        pt.setY(y);
        pose.getPose().setPosition(pt);
        System.out.println("Goal Heading: "+AbstractLocaliser.getHeading(pose.getPose().getOrientation())
                + "   x " + pose.getPose().getPosition().getX()
                + "   y " + pose.getPose().getPosition().getY());
        return pose;
    }
}
