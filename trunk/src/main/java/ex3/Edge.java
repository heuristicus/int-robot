package ex3;

public class Edge {

    Vertex v1;
    Vertex v2;
    double weight;

    /*
     * Creates an edge connecting two vertices.
     */
    public Edge(Vertex v1, Vertex v2){
        this.v1 = v1;
        this.v2 = v2;
    }

    public double edgeWeight(){
        return weight;
    }

    public void setEdgeWeight(double edgeWeight){
        this.weight = edgeWeight;
    }
    

}
