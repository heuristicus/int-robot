package ex3;

import geometry_msgs.Point;
import java.util.ArrayList;
import nav_msgs.OccupancyGrid;

public class PRMGraph {

    ArrayList<Vertex> vertices;
    ArrayList<Edge> edges;
    double distanceThreshold;

    public PRMGraph(PRMUtil util, OccupancyGrid map, int numVertices, double distanceThreshold){
        this.distanceThreshold = distanceThreshold;
        generatePRM(util, map, numVertices);
    }

    /*
     * Generates the road map using the provided map and vertex number.
     */
    public void generatePRM(PRMUtil util, OccupancyGrid map, int numVertices){
        vertices = util.generateRandomVertices(map, numVertices);
        this.edges = util.connectVertices(vertices, distanceThreshold);
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
