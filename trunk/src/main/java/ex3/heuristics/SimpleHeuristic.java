package ex3.heuristics;

import ex3.PRMUtil;
import ex3.Vertex;

public class SimpleHeuristic implements Heuristic {

    @Override
    public double evaluate(Vertex v1, Vertex v2) {
        return PRMUtil.getEuclideanDistance(v1, v2);
    }

}
