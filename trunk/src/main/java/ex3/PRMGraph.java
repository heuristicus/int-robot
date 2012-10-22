package ex3;

import ex3.search.SearchAlgorithm;
import java.util.ArrayList;
import nav_msgs.OccupancyGrid;

public class PRMGraph {

    ArrayList<Vertex> vertices;
    ArrayList<Edge> edges;

    public PRMGraph(PRMUtil util, OccupancyGrid map, int numVertices){
        generatePRM(util, map, numVertices);
    }

    /*
     * Generates the road map using the provided map and vertex number.
     */
    public void generatePRM(PRMUtil util, OccupancyGrid map, int numVertices){
        vertices = util.generateRandomVertices(map, numVertices);
        edges = util.connectVertices(vertices);
    }

    
}
