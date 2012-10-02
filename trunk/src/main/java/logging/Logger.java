package logging;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;

public class Logger extends AbstractNodeMain{

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Logger");
    }

    @Override
    public void onShutdown(Node node) {
        System.out.println("Node " + this.getDefaultNodeName() + " successfully shut down.");
    }

    

}