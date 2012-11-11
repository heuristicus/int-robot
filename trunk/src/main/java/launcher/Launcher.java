package launcher;

import ex3.PRM;
import ex3.navigation.Navigator;
import ex3.search.Dijkstra;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class Launcher extends AbstractNodeMain {

    static {
        System.setProperty("org.apache.commons.logging.Log",
                         "org.apache.commons.logging.impl.NoOpLog");
    }

    // Create a default configuration for the nodes that we will be creating
    NodeConfiguration conf = NodeConfiguration.newPrivate();
    // Create an executor to use to execute nodes.
    NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Start up the specified nodes
        //exec.execute(new Logger(), conf);
        //exec.execute(new Avoider2(), conf);
//        PFLocalisationNode.experimentMode = true;
//        PFLocalisationNode.realWorldMode = false;
//        PFLocalisationNode.augmented = true;
//        exec.execute(new PFLocalisationNode(), conf);

        PRM prm = new PRM(new Dijkstra(), false);
        exec.execute(prm, conf);
        exec.execute(new Navigator(prm), conf);

//        PID pid = new PID(0.5, 0.5, 0.5, 1.0, 1.0, PID.DIRECTION.DIRECT);
//        exec.execute(pid, conf);
//        exec.execute(new Navigator(prm, pid), conf);


//        try {
//            BatchExperimenter.runAllExperiments(connectedNode);
//        } catch (Exception e) {
//            System.out.println("Couldn't create BatchExperimenter");
//            e.printStackTrace();
//        }

//        exec.execute(new ExperimentNav(PFLocalisationNode.realWorldMode), conf);
    }

    @Override
    public void onShutdown(Node node) {
        // Shut down all the nodes that we've started up.
        System.out.println("Launcher shutting down started nodes...");
        exec.shutdown();
        System.out.println("All node shutdowns complete.");
    }
    
    // Doesn't seem like this is being called?
    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("Launcher Shutting down...");
        System.out.println("Node " + this.getDefaultNodeName() + " successfully shut down.");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Launcher");
    }

}