package ex3;

import ex3.search.SearchAlgorithm;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import nav_msgs.OccupancyGrid;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud;
import visualization_msgs.Marker;

public class PRM extends AbstractNodeMain {

    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    public static ConnectedNode node;
    OccupancyGrid map;
    public static final int NUMBER_OF_VERTICES = 400;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 2.0;

    Subscriber<OccupancyGrid> grid;
    Publisher<PointCloud> roadMap;
    Publisher<Marker> lines;

    public PRM(){
        
    }

    public PRM(SearchAlgorithm search){
        this.search = search;
    }

        @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("PRMnode");
    }

    @Override
    public void onStart(final ConnectedNode node) {
        grid = node.newSubscriber("map", OccupancyGrid._TYPE);
        roadMap = node.newPublisher("PRMGraph", PointCloud._TYPE);
        PRM.node = node;

        final MessageFactory factory = node.getTopicMessageFactory();
        grid.addMessageListener(new MessageListener<OccupancyGrid>() {
            @Override
            public void onNewMessage(OccupancyGrid message) {
                System.out.println("PRM node received map. Initialising road map.");
                map = message;
                mapReceived = true;
                initialisePRM(factory);
            }
        });

    }

    /* Initialises the PRM with a utility object and a graph. */
    public void initialisePRM(MessageFactory factory) {
       
        util = new PRMUtil(new Random(), factory);
        graph = new PRMGraph(util, map, NUMBER_OF_VERTICES, PROXIMITY_DISTANCE_THRESHOLD);
        PointCloud p = factory.newFromType(PointCloud._TYPE);
        p.setPoints(graph.getVertexLocations());
        p.getHeader().setFrameId("/map");

        try {
            roadMap.publish(p);
            Thread.sleep(200);
            roadMap.publish(p);
            Thread.sleep(200);
            roadMap.publish(p);
        } catch (InterruptedException ex) {
            Logger.getLogger(PRM.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    /* Sets the search algorithm to be used by the graph to search for the
     * shortest path from one vertex in the graph to another */
    public void setSearchAlgorithm(SearchAlgorithm searchAlg) {
        this.search = searchAlg;
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("PRM node successfully shut down.");
    }





}
