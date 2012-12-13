package ex3.search;

import ex3.PRMGraph;
import util.PRMUtil;
import ex3.Vertex;
import java.util.ArrayList;

public interface SearchAlgorithm {

    /* Calculate the shortest path through the graph from one vertex to another. */
    public ArrayList<Vertex> shortestPath(Vertex v1, Vertex v2, PRMGraph graph, PRMUtil util);

}

