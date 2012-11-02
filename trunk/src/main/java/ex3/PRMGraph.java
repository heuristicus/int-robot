package ex3;

import geometry_msgs.Point;
import java.util.ArrayList;
import nav_msgs.OccupancyGrid;

public class PRMGraph {

    public final int MAX_CONNECTIONS;

    ArrayList<Vertex> vertices;
    ArrayList<Edge> edges;
    double distanceThreshold;

    public PRMGraph(PRMUtil util, OccupancyGrid map, int numVertices, double distanceThreshold, int max_connections){
        this.MAX_CONNECTIONS = max_connections;
        this.distanceThreshold = distanceThreshold;
    }

    /* Generates the road map using the provided map and vertex number. */
    public void generatePRM(PRMUtil util, OccupancyGrid map, int numVertices){
        //vertices = util.randomSample(map, numVertices);
        //vertices = util.gridSample(map, 0.5, 0.5);
        vertices = util.cellSample(map, 1.0, 5);
        long start = System.currentTimeMillis();
        this.edges = util.connectVertices(vertices, distanceThreshold, MAX_CONNECTIONS);
        System.out.println("All vertices connected in: "+(System.currentTimeMillis()-start)+" ms");
    }

    public boolean addVertex(Vertex v, PRMUtil util){
        if (vertices.contains(v)){
            // If we've added the vertex before.
            return false;
        }
        vertices.add(v);

        edges.addAll(util.connectVertexToGraph(v, vertices, distanceThreshold, MAX_CONNECTIONS));

        return true;
    }

    public ArrayList<Edge> getEdges() {
        return edges;
    }

    public ArrayList<Vertex> getVertices() {
        return vertices;
    }

    public ArrayList<Point> getVertexLocations(){
        ArrayList<Point> points = new ArrayList<Point>();
        for (Vertex vertex : vertices) {
            points.add(vertex.getLocation());
        }
        return points;
    }

}
