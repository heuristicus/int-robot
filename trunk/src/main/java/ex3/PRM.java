package ex3;

import ex3.search.SearchAlgorithm;
import java.util.LinkedList;
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
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

public class PRM extends AbstractNodeMain {

    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    public static ConnectedNode node;
    OccupancyGrid map;
    OccupancyGrid inflatedMap;
    public static final int NUMBER_OF_VERTICES = 50;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 3.5;

    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> PRMMarkers;
    Publisher<Marker> pathMarkers;
    Publisher<OccupancyGrid> inflatedMapPublisher;

    public PRM(){}

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
        PRMMarkers = node.newPublisher("markers", MarkerArray._TYPE);
        PRMMarkers.setLatchMode(true);
        pathMarkers = node.newPublisher("pathMarkers", Marker._TYPE);
        pathMarkers.setLatchMode(true);
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
        inflatedMap = util.inflateMap(map, inflatedMapPublisher);
        util = new PRMUtil(new Random(), factory, inflatedMap);
        graph = new PRMGraph(util, inflatedMap, NUMBER_OF_VERTICES, PROXIMITY_DISTANCE_THRESHOLD);

        publishMarkers(graph);
        inflatedMapPublisher.setLatchMode(true);
        inflatedMapPublisher.publish(inflatedMap);

        System.out.println("Average path length: " + util.averageConnectionLength(graph));

        int start = (int) (Math.random() * NUMBER_OF_VERTICES);
        int goal = (int) (Math.random() * NUMBER_OF_VERTICES);

        LinkedList<Vertex> rt = findRoute(graph.vertices.get(start), graph.getVertices().get(goal));

        if (rt.size() == 0){
            System.out.println("Could not find route!");
        } else {
            pathMarkers.publish(util.makePathMarker(rt));
        }



    }

    public LinkedList<Vertex> findRoute(Vertex v1, Vertex v2){
        return search.shortestPath(v1, v2, graph, util);
    }

    public void publishMarkers(PRMGraph graph){
        MarkerArray array = PRMMarkers.newMessage();

        array.setMarkers(util.getGraphMarkers(graph, inflatedMap, "/map"));

        for (int i = 0; i < 10; i++) {
            PRMMarkers.publish(array);
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
