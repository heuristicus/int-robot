package launcher;

import ex1.Avoider2;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;


public class Launcher extends AbstractNodeMain {

    @Override
    public void onStart(ConnectedNode connectedNode) {
        NodeConfiguration conf = NodeConfiguration.newPrivate();
        NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();
        exec.execute(new Avoider2(), conf);

    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Launcher");
    }

}