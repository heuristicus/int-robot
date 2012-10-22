package ex3;

import ex3.search.Dijkstra;
import ex3.search.SearchAlgorithm;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

public class PRM extends AbstractNodeMain{

    PRMUtil util;
    PRMGraph graph;
    SearchAlgorithm search;
    boolean mapReceived = false;
    OccupancyGrid map;
    public static final int NUMBER_OF_VERTICES = 300;

    public PRM(SearchAlgorithm search){
        this.search = search;
    }

    @Override
    public void onStart(ConnectedNode node) {
        Subscriber<OccupancyGrid> grid = node.newSubscriber("map", OccupancyGrid._TYPE);
        grid.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid message) {
                System.out.println("PRM node received map. Initialising road map.");
                map = message;
                mapReceived = true;
                initialisePRM();
            }
        });
    }

    /*
     * Initialises the PRM with a utility object and a graph.
     */
    public void initialisePRM(){
        util = new PRMUtil(new Random());
        graph = new PRMGraph(util, map, NUMBER_OF_VERTICES);
    }

    /*
     * Sets the search algorithm to be used by the graph to search for the shortest
     * path from one vertex in the graph to another
     */
    public void setSearchAlgorithm(SearchAlgorithm searchAlg){
        this.search = searchAlg;
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("PRM node successfully shut down.");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("PRM node");
    }



}
