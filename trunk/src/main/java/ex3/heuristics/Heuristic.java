package ex3.heuristics;

import ex3.Edge;

public interface Heuristic {

    /* Evaluate the utility of the given edge. */
    public abstract double evaluate(Edge e);

}
