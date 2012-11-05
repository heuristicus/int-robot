package ex3.experiment;

import ex3.PRM;
import ex3.PRMUtil;
import ex3.Vertex;
import ex3.search.Dijkstra;
import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import launcher.RunParams;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import pf.AbstractLocaliser;
import visualization_msgs.MarkerArray;

public class BatchExperimenter {

    private static Publisher<PoseWithCovarianceStamped> initialPosePub;
    private static Publisher<PoseStamped> goalPub;

    private static PRM prm;
//    private static int valueArrayIndex;
//    private static String[] valuesArray;
    private static ConnectedNode node;
    private static int expNum;
    /** Maps minor expNums to lists of values that will vary for each minor exp */
    private static List<List<String>> expValues;
    private static int[] expMinorNums;
    private static String key;

    public static void runAllExperiments(ConnectedNode connectedNode) {
        System.out.println("runAllExperiments called");
        node = connectedNode;

        initialPosePub = node.newPublisher("initialpose", PoseWithCovarianceStamped._TYPE);
//        initialPosePub.setLatchMode(true);
        goalPub = node.newPublisher("goal", PoseStamped._TYPE);
//        goalPub.setLatchMode(true);

        Subscriber<MarkerArray> routeSub = connectedNode.newSubscriber("pathMarkers", MarkerArray._TYPE);
        routeSub.addMessageListener(new MessageListener<MarkerArray>() {
            @Override
            public void onNewMessage(MarkerArray t) {
                processPath();
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
        System.out.println("creating new prm");
        prm = new PRM(new Dijkstra());
        System.out.println("executing prm");
        exec.execute(prm, conf);

        try {
            System.out.println("waiting for PRM to initialise");
            while(!prm.graphGenerationComplete()){
                System.out.print(".");
                Thread.sleep(200);
            }
            System.out.println("");
        } catch (InterruptedException e) {
        }

        System.out.println("constructing initial pose message");
        PoseWithCovarianceStamped initialPose = initialPosePub.newMessage();
        initialPose = getStartPose(initialPose, node.getTopicMessageFactory(), expNum);
        initialPosePub.publish(initialPose);
        System.out.println("sent initial pose message");

        try {
            System.out.println("waiting for pose message receipt");
            while (prm.getCurrentPosition() == null){
                System.out.print(".");
                Thread.sleep(200);
            }
            System.out.println("");
        } catch (InterruptedException e) {
        }

        System.out.println("constructing goal pose message");
        PoseStamped goal = goalPub.newMessage();
        goal = getGoalPose(goal, node.getTopicMessageFactory(), expNum);
        goalPub.publish(goal);
        System.out.println("sent goal message");
    }

    public static void processPath(){
        ArrayList<Vertex> routeToGoal = prm.getRouteToGoal();
        double length = PRMUtil.getPathLength(routeToGoal);
        System.out.println("Experiment: Path of length " + length + " found and recorded.");
        System.out.println("Moreover, regeneration attempts: "+prm.getRegenerationsForLastSearch());

        boolean moreMinorExperiments = incrementMinorNums(expMinorNums.length-1);
        if (! moreMinorExperiments) {
            expNum++;
            System.out.println("No more minor experiments. Setting expNum to: "+expNum);
            boolean experimentsToDo = initialiseExperiments(expNum);
            if (! experimentsToDo) {
                System.out.println("No more experiments to do. Goodbye cruel world.");
                System.exit(0);
            }
        }

        runNextExperiment();
    }

    public static PoseWithCovarianceStamped getStartPose(PoseWithCovarianceStamped pose, MessageFactory factory, int expNum) {
        AbstractLocaliser.setFactory(factory);
        pose.getPose().getPose().setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 2.321)); //approximate
        Point pt = factory.newFromType(Point._TYPE);
        double x = RunParams.getDouble("EXPERIMENT_"+expNum+"_STARTX");
        double y = RunParams.getDouble("EXPERIMENT_"+expNum+"_STARTY");
        pt.setX(x);
        pt.setY(y);
        pose.getPose().getPose().setPosition(pt);
        System.out.println("Heading: "+AbstractLocaliser.getHeading(pose.getPose().getPose().getOrientation())
                + "   x " + pose.getPose().getPose().getPosition().getX()
                + "   y " + pose.getPose().getPose().getPosition().getY());
        return pose;
    }

    public static PoseStamped getGoalPose(PoseStamped pose, MessageFactory factory, int expNum) {
        pose.getPose().setOrientation(AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), 2.321)); //approximate
        Point pt = factory.newFromType(Point._TYPE);
        double x = RunParams.getDouble("EXPERIMENT_" + expNum + "_GOALX");
        double y = RunParams.getDouble("EXPERIMENT_"+expNum+"_GOALY");
        pt.setX(x);
        pt.setY(y);
        pose.getPose().setPosition(pt);
        System.out.println("Heading: "+AbstractLocaliser.getHeading(pose.getPose().getOrientation())
                + "   x " + pose.getPose().getPosition().getX()
                + "   y " + pose.getPose().getPosition().getY());
        return pose;
    }
}
