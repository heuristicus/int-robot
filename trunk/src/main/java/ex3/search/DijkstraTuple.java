package ex3.search;

import ex3.Vertex;

/*
 * Modifiable tuple for storing the distance and previous node, for use with
 * Dijkstra's algorithm.
 */
public class DijkstraTuple {

    private double distance;
    private Vertex previous;

    public DijkstraTuple(double distance, Vertex previous){
        this.distance = distance;
        this.previous = previous;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public Vertex getPrevious() {
        return previous;
    }

    public void setPrevious(Vertex previous) {
        this.previous = previous;
    }
    

}
