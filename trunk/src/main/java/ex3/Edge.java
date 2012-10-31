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
    @Override
    public boolean equals(Object obj) {
        try {
            Edge e = (Edge) obj;
            return (e.getVertexA().equals(this.a) && e.getVertexB().equals(this.b)) || (e.getVertexA().equals(this.b) && e.getVertexB().equals(this.a));
        } catch (ClassCastException ex){
            return false;
        }

    }

    @Override
    public String toString() {
        return "Point a: " + this.a.toString() + ", Point b: " + this.b.toString() + "Weight: " + this.weight;
    }



}
