package ex3.experiment;

import ex3.PRM;
import ex3.PRMUtil;
import ex3.Vertex;
import ex3.search.Dijkstra;
import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import java.util.ArrayList;
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
    private static int valueArrayIndex;
    private static String[] valuesArray;
    private static ConnectedNode node;
    private static int expNum;
    private static String key;

    public static void runAllExperiments(ConnectedNode connectedNode) {
        System.out.println("runAllExperiments called");
        node = connectedNode;
        expNum = 0;

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
        runNextExperiment();
    }
    
    public static void runNextExperiment() {
        expNum++;
        key = RunParams.get("EXPERIMENT_" + expNum + "_KEY");
        System.out.println("Experimenting with: EXPERIMENT_" + expNum + "_KEY");
        if (key == null) {
            System.out.println("No more experiments in Params file. Exiting");
            System.exit(0);
        }

        String expSampling = RunParams.get("EXPERIMENT_" + expNum + "_SAMPLING");
        RunParams.overrideProperty("SAMPLING_METHOD", expSampling);

        valuesArray = RunParams.get("EXPERIMENT_" + expNum + "_VALUES").split(",");
        runExperimentOverValues();
    }

    // Create a default configuration for the nodes that we will be creating
    private static NodeConfiguration conf = NodeConfiguration.newPrivate();
    // Create an executor to use to execute nodes.
    private static NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();

    public static void runExperimentOverValues() {
        valueArrayIndex = 0;
        runExperimentForValue(key, valuesArray[0]);
    }

    public static void runExperimentForValue(String key, String value) {
        System.out.println("Overriding property " + key + "=" + value);
        System.out.println("Value of num_vertices before override: " + RunParams.getInt("NUMBER_OF_VERTICES"));
        RunParams.overrideProperty(key, value);
        System.out.println("Value of num_vertices after override: " + RunParams.getInt("NUMBER_OF_VERTICES"));

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
        valueArrayIndex++;
        if (valueArrayIndex < valuesArray.length){
            runExperimentForValue(key, valuesArray[valueArrayIndex]);
        } else {
            runNextExperiment();
        }
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
