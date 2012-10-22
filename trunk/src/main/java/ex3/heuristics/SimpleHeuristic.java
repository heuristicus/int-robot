package ex3.heuristics;

import ex3.Edge;

public class SimpleHeuristic implements Heuristic {

    @Override
    public double evaluate(Edge e) {
        return e.edgeWeight();
    }

}
