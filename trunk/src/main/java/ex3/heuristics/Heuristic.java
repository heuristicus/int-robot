package ex3.heuristics;

import ex3.Vertex;

public interface Heuristic {

    public abstract double evaluate(Vertex v1, Vertex v2);

}
