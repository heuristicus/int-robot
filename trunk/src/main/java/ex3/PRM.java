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
    public static final int NUMBER_OF_VERTICES = 100;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 3.0;

    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> markers;

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

//    public void publishLines_arrow(MessageFactory factory) {
//        ArrayList<Marker> markers = new ArrayList<Marker>();
//        Marker m = factory.newFromType(Marker._TYPE);
//
//        m.getHeader().setFrameId("/map");
//        m.getHeader().setStamp(node.getCurrentTime());
//
//        m.setNs("edges");
//        m.setId(0);
//        m.setType(Marker.ARROW);
//        m.setAction(m.ADD);
//
//
//        m.getPose().getPosition().setX(20);
//        m.getPose().getPosition().setY(30);
//        m.getPose().getPosition().setZ(0.0);
//        m.getPose().getOrientation().setW(1.0);
//        m.getPose().getOrientation().setX(0.0);
//        m.getPose().getOrientation().setY(0.0);
//        m.getPose().getOrientation().setZ(0.0);
//
//        m.getScale().setX(20);
//        m.getScale().setY(5);
//        m.getScale().setZ(0);
//
//        m.getColor().setA(1.0f);
//        m.getColor().setR(1.0f);
//        m.setLifetime(Duration.fromMillis(100000));
//
//
//        this.markers.publish(m);
//
//    }

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
