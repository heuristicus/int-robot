package util;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.message.p2os_driver.SonarArray;
import org.ros.node.topic.Subscriber;

public class SonarUtil extends AbstractNodeMain {

    Subscriber<SonarArray> sonar;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SonarUtil");
    }


}
