package launcher;

import ex3.PRM;
import ex3.search.Dijkstra;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class Launcher extends AbstractNodeMain {

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
        exec.execute(new PRM(new Dijkstra()), conf);
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