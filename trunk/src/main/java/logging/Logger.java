package logging;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;

public class Logger extends AbstractNodeMain{

    private final String logFile;

    public Logger(){
        Calendar curDate = Calendar.getInstance();
        DateFormat form = new SimpleDateFormat("dd/MM/yyyy_HH-mm-ss");
        logFile = "run_log_" + form.format(curDate.getTime()) + ".log";
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        System.out.println("Logging to " + logFile + "...");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Logger");
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("Node " + this.getDefaultNodeName() + " successfully shut down.");
    }

}