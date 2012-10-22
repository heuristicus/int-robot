package ex3.search;

import ex3.Edge;
import ex3.PRMGraph;
import ex3.Vertex;
import ex3.heuristics.Heuristic;
import java.util.ArrayList;

public class Dijkstra implements SearchAlgorithm {

    Heuristic heuristic;

    public Dijkstra(Heuristic h){
        this.heuristic = h;
    }

    @Override
    public ArrayList<Edge> shortestPath(Vertex v1, Vertex v2, PRMGraph graph) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

}
