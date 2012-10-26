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
import visualization_msgs.MarkerArray;

public class PRM extends AbstractNodeMain {

    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    public static ConnectedNode node;
    OccupancyGrid map;
    OccupancyGrid inflatedMap;
    public static final int NUMBER_OF_VERTICES = 100;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 3.0;

    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> markers;
    Publisher<OccupancyGrid> inflatedMapPublisher;

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
        inflatedMapPublisher = node.newPublisher("inflatedMap", OccupancyGrid._TYPE);
        markers = node.newPublisher("markers", MarkerArray._TYPE);
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

        publishMarkers(graph);
        inflatedMap = util.inflateMap(map, inflatedMapPublisher);
        inflatedMapPublisher.setLatchMode(true);
        inflatedMapPublisher.publish(inflatedMap);
    }

    public void publishMarkers(PRMGraph graph){
        MarkerArray array = markers.newMessage();

        array.setMarkers(util.getGraphMarkers(graph, "/map"));

        for (int i = 0; i < 10; i++) {
            markers.publish(array);
            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                Logger.getLogger(PRM.class.getName()).log(Level.SEVERE, null, ex);
            }
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
