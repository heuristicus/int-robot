package ex3.search;

import ex3.Edge;
import ex3.PRMGraph;
import ex3.Vertex;
import java.util.ArrayList;

public interface SearchAlgorithm {

    /* Calculate the shortest path through the graph from one vertex to another. */
    public ArrayList<Edge> shortestPath(Vertex v1, Vertex v2, PRMGraph graph);

}

