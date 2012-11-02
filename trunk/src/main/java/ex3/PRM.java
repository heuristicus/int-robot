package ex3;

import ex3.search.SearchAlgorithm;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import java.util.ArrayList;
import java.util.List;
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

    public static final int NUMBER_OF_VERTICES = 500;
    public static final double PROXIMITY_DISTANCE_THRESHOLD = 3.0;
    public final int MAX_CONNECTIONS = 5;

    public final int MAX_REGENERATION_ATTEMPTS = 200;
    
    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    public static ConnectedNode node;
    OccupancyGrid map;
    OccupancyGrid inflatedMap;

    ArrayList<Vertex> routeToGoal;
    Pose currentPosition;
    
    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> PRMMarkers;
    Publisher<MarkerArray> pathMarkers;
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
        pathMarkers = node.newPublisher("pathMarkers", MarkerArray._TYPE);
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
                
                int attempts = 0;
                routeToGoal = null; // Start with no path
                // Try to find a path or regenerate graph until able to
                while (routeToGoal == null && attempts < MAX_REGENERATION_ATTEMPTS) {
                    System.out.println("Finding route attempt: "+ ++attempts);

                    Vertex start = new Vertex(currentPosition.getPosition());
                    Vertex goal = new Vertex(t.getPose().getPosition());

                    // Try and add the start and goal points to the graph
                    boolean startAdded = graph.addVertex(start, util);
                    boolean goalAdded = graph.addVertex(goal, util);

                    if (!startAdded) {
                        start = graph.getVertices().get(graph.getVertices().indexOf(start));
                    } else {
                        System.out.println("Start point added to the graph.");
                    }
                    if (!goalAdded) {
                        goal = graph.getVertices().get(graph.getVertices().indexOf(goal));
                    } else {
                        System.out.println("Goal point added to the graph");
                    }
                    routeToGoal = findRoute(start, goal);
                }

                if (routeToGoal == null) {
                    System.out.println("Could not find a path. Are you sure "
                            + "the destination is reachable?");
                    return;
                }

                System.out.println("Found route of size: " + routeToGoal.size() + ". Flattening...");
                List flatRouteToGoal = util.flattenDrunkenPath(routeToGoal, -1); // -1 is flatten fully
                System.out.println("Flattened to size: " + flatRouteToGoal.size());
                double percentage = (double) flatRouteToGoal.size() / (double) routeToGoal.size();
                System.out.printf("New path is %.2f times the size of the original.\n", percentage);
                MarkerArray paths = pathMarkers.newMessage();
                ArrayList<Marker> pathList = new ArrayList<Marker>();
                pathList.add(util.makePathMarker(routeToGoal, "originalPath", "blue"));
                pathList.add(util.makePathMarker(flatRouteToGoal, "flattenedPath", "green"));

                paths.setMarkers(pathList);
                pathMarkers.setLatchMode(true);
                pathMarkers.publish(paths);
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
        graph.generatePRM(util, inflatedMap, NUMBER_OF_VERTICES);

        publishMarkers(graph);
        inflatedMapPublisher.setLatchMode(true);
        inflatedMapPublisher.publish(inflatedMap);

        System.out.println("Average path length: " + util.averageConnectionLength(graph));

    }

    public ArrayList<Vertex> findRoute(Vertex v1, Vertex v2){
        double start = System.currentTimeMillis();
        ArrayList<Vertex> path =  search.shortestPath(v1, v2, graph, util);
        System.out.println(search + " search took " + (System.currentTimeMillis() - start) + "ms.");
        return path;
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
