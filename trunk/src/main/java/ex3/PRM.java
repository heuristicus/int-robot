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
import launcher.RunParams;
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

    public int MAX_REGENERATION_ATTEMPTS = RunParams.getInt("MAX_REGENERATION_ATTEMPTS");//200;
    
    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    boolean graphGenerationComplete = false;
    private int regenerationAttempts = 0;
    public static ConnectedNode node;
    OccupancyGrid map;
    OccupancyGrid inflatedMap;
    private boolean experimentMode = false;

    ArrayList<Vertex> routeToGoal;
    ArrayList<Vertex> flatRouteToGoal;
    Pose currentPosition;
    Pose goalPosition;
    
    Subscriber<OccupancyGrid> grid;
    Publisher<MarkerArray> PRMMarkers;
    Publisher<MarkerArray> pathMarkers;
    Publisher<OccupancyGrid> inflatedMapPublisher;
    Subscriber<PoseStamped> goals;
    Subscriber<PoseWithCovarianceStamped> initialPosition;

    public PRM(SearchAlgorithm search, boolean experimentMode){
        this.search = search;
        this.experimentMode = experimentMode;
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
        pathMarkers = node.newPublisher("pathMarkers", MarkerArray._TYPE);
        if (! experimentMode) {
            PRMMarkers.setLatchMode(true);
        }
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
                if (inFreeSpace){
                    goalPosition = t.getPose();
                } else {
                    System.out.println("Cannot move to specified location. In a wall or outside the map.");
                    return; // If the pose is not in free space, we reject this goal.
                }

                generateAndPublishRouteToGoal();
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

    /** Generates route to goal, regenerating the graph if necessary. */
    public ArrayList<Vertex> generateAndPublishRouteToGoal() {
        regenerationAttempts = 0;
        routeToGoal = null; // Start with no path
        // Try to find a path or regenerate graph until able to
        while (routeToGoal == null && regenerationAttempts < MAX_REGENERATION_ATTEMPTS) {
            System.out.println("Finding route attempt: " + ++regenerationAttempts);

            if (regenerationAttempts > 1) {
                // Regenerate graph
                graph.generatePRM(util, inflatedMap);
            }

            Vertex start = new Vertex(currentPosition.getPosition());
            Vertex goal = new Vertex(goalPosition.getPosition());

            // Try and add the start and goal points to the graph
            boolean startAdded = graph.addVertex(start, util);
            boolean goalAdded = graph.addVertex(goal, util);

            if (!startAdded) {
                start = graph.getVertices().get(graph.getVertices().indexOf(start));
            } else {
                System.out.println("Start point added to the graph: "
                        + start.getLocation().getX() + ", " + start.getLocation().getY());
            }
            if (!goalAdded) {
                goal = graph.getVertices().get(graph.getVertices().indexOf(goal));
            } else {
                System.out.println("Goal point added to the graph: "
                        + goal.getLocation().getX() + ", " + goal.getLocation().getY());
            }

            System.out.println("Publishing markers for graph...");
            publishMarkers(graph);
            System.out.println("Finding route to goal...");
            routeToGoal = findRoute(start, goal);
        }

        if (routeToGoal == null) {
            System.out.println("Could not find a path. Are you sure "
                    + "the destination is reachable?");
            MarkerArray paths = pathMarkers.newMessage();
            ArrayList<Marker> pathList = new ArrayList<Marker>();
            paths.setMarkers(pathList);
            pathMarkers.publish(paths);
            routeToGoal = new ArrayList<Vertex>();
            return null;
        }

        System.out.println("Found route of size: " + routeToGoal.size() + ". Flattening...");
        flatRouteToGoal = util.flattenDrunkenPath(routeToGoal, -1); // -1 is flatten fully
        System.out.println("Flattened to size: " + flatRouteToGoal.size());
        double percentage = (double) flatRouteToGoal.size() / (double) routeToGoal.size();
        System.out.printf("New path is %.2f times the size of the original.\n", percentage);
        MarkerArray paths = pathMarkers.newMessage();
        ArrayList<Marker> pathList = new ArrayList<Marker>();
        pathList.add(util.makePathMarker(routeToGoal, "originalPath", "blue"));
        pathList.add(util.makePathMarker(flatRouteToGoal, "flattenedPath", "orange"));

        paths.setMarkers(pathList);
        pathMarkers.publish(paths);
        return flatRouteToGoal;
    }

    public Pose getCurrentPosition() {
        return currentPosition;
    }

    public Pose getGoalPosition() {
        return goalPosition;
    }

    public void setCurrentPosition(Pose currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void setGoalPosition(Pose goalPosition) {
        this.goalPosition = goalPosition;
    }

    /** How many times we regenerated the graph the last time we searched
     * for a path. */
    public int getRegenerationsForLastSearch() {
        return regenerationAttempts;
    }

        /* Initialises the PRM with a utility object and a graph. */
    public void initialisePRM(MessageFactory factory) {
        inflatedMap = util.inflateMap(map, inflatedMapPublisher);
        util = new PRMUtil(new Random(), factory, inflatedMap);
        graph = new PRMGraph();
        graph.generatePRM(util, inflatedMap);

        publishMarkers(graph);
        if (! experimentMode) {
            inflatedMapPublisher.setLatchMode(true);
        }
        inflatedMapPublisher.publish(inflatedMap);

        System.out.println("Average path length: " + util.averageConnectionLength(graph));
        graphGenerationComplete = true;
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
        PRMMarkers.publish(array);
    }

    public ArrayList<Vertex> getRouteToGoal() {
        return routeToGoal;
    }

    /* Sets the search algorithm to be used by the graph to search for the
     * shortest path from one vertex in the graph to another */
    public void setSearchAlgorithm(SearchAlgorithm searchAlg) {
        this.search = searchAlg;
    }

    public boolean graphGenerationComplete() {
        return graphGenerationComplete;
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("PRM node successfully shut down.");
    }

}
