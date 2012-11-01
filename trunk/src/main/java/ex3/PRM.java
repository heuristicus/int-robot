package ex3;

import ex3.search.SearchAlgorithm;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
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

    public static final int NUMBER_OF_VERTICES = 1000;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 1.0;
    public final int MAX_CONNECTIONS = 5;
    
    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    public static ConnectedNode node;
    OccupancyGrid map;
    OccupancyGrid inflatedMap;

    LinkedList<Vertex> routeToGoal;
    Pose currentPosition;
    
    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> PRMMarkers;
    Publisher<Marker> pathMarkers;
    Publisher<OccupancyGrid> inflatedMapPublisher;
    Subscriber<PoseStamped> goals;
    Subscriber<PoseWithCovarianceStamped> initialPosition;

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
        goals = node.newSubscriber("goal", PoseStamped._TYPE);
        initialPosition = node.newSubscriber("initialpose", PoseWithCovarianceStamped._TYPE);
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

        goals.addMessageListener(new MessageListener<PoseStamped>() {

            @Override
            public void onNewMessage(PoseStamped t) {

                if (currentPosition == null) {
                    System.out.println("Starting position unset!");
                    return;
                }

                boolean inFreeSpace = util.checkPositionValidity(t.getPose(), inflatedMap);

                if (!inFreeSpace){
                    System.out.println("Cannot move to specified location. In a wall or outside the map.");
                    return; // If the pose is not in free space, we reject this goal.
                }
                
                Vertex start = new Vertex(currentPosition.getPosition());
                Vertex goal = new Vertex(t.getPose().getPosition());

                // Try and add the start point to the graph
                boolean startAdded = graph.addVertex(start, util);

                // Try and add the goal point to the graph
                boolean goalAdded = graph.addVertex(goal, util);

                if (!startAdded){
                    System.out.println("Start point is already in the graph.");
                    start = graph.getVertices().get(graph.getVertices().indexOf(start));
                }

                if (!goalAdded){
                    System.out.println("Goal point is already in the graph");
                    goal = graph.getVertices().get(graph.getVertices().indexOf(goal));
                }

                routeToGoal = findRoute(start, goal);
                if (routeToGoal == null){
                    System.out.println("Could not find route!");
                } else {
                    pathMarkers.publish(util.makePathMarker(routeToGoal));
                }

            }
        });

        initialPosition.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {

            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {

                boolean inFreeSpace = util.checkPositionValidity(t.getPose().getPose(), inflatedMap);

                if (inFreeSpace){
                    currentPosition = t.getPose().getPose();
                    System.out.println("PRM Starting position received.");
                } else {
                    System.out.println("Cannot start inside a wall or outside the map.");
                }

            }
        });

    }

    /* Initialises the PRM with a utility object and a graph. */
    public void initialisePRM(MessageFactory factory) {
        inflatedMap = util.inflateMap(map, inflatedMapPublisher);
        util = new PRMUtil(new Random(), factory, inflatedMap);
        graph = new PRMGraph(util, inflatedMap, NUMBER_OF_VERTICES, PROXIMITY_DISTANCE_THRESHOLD, MAX_CONNECTIONS);

        publishMarkers(graph);
        inflatedMapPublisher.setLatchMode(true);
        inflatedMapPublisher.publish(inflatedMap);

        System.out.println("Average path length: " + util.averageConnectionLength(graph));

//        int start = (int) (Math.random() * NUMBER_OF_VERTICES);
//        int goal = (int) (Math.random() * NUMBER_OF_VERTICES);
//
//        LinkedList<Vertex> rt = findRoute(graph.vertices.get(start), graph.getVertices().get(goal));
//
//        if (rt == null){
//            System.out.println("Could not find route!");
//        } else {
//            pathMarkers.publish(util.makePathMarker(rt));
//        }

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
