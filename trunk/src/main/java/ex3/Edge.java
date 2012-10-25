package ex3;

public class Edge {

    Vertex a;
    Vertex b;
    double weight;

    /*
     * Creates an edge connecting two vertices.
     */
    public Edge(Vertex a, Vertex b, double weight){
        this.a = a;
        this.b = b;
        this.weight = weight;
    }

    public double edgeWeight(){
        return weight;
    }

    public void setEdgeWeight(double edgeWeight){
        this.weight = edgeWeight;
    }
    
    public Vertex getVertexA(){
        return a;
    }
    
    public Vertex getVertexB(){
        return b;
    }

    /*
     * Checks if two edges are equal for edges e1 and e2. Equality is defined as follows:
     * e1.a == e2.a && e1.b == e2.b
     * or, e1.a == e2.b && e1.b == e2.a
     * A path that starts at a and ends at b is equivalent to one that starts at
     * b and ends at a.
     */
    public boolean isEqual(Edge e){
        return (e.getVertexA().isEqual(this.a) && e.getVertexB().isEqual(this.b)) || (e.getVertexA().isEqual(this.b) && e.getVertexB().isEqual(this.a));
    }
    
}
