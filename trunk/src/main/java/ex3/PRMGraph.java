package ex3;

import geometry_msgs.Point;
import java.util.ArrayList;
import launcher.RunParams;
import nav_msgs.OccupancyGrid;

public class PRMGraph {

    public int NUMBER_OF_VERTICES = RunParams.getInt("NUMBER_OF_VERTICES");
    public int TARGET_PER_CELL = RunParams.getInt("TARGET_PER_CELL");//5;
    public double CELL_WIDTH = RunParams.getDouble("CELL_WIDTH");//1.0;
    public double X_STEP = RunParams.getDouble("X_STEP");//5;
    public double Y_STEP = RunParams.getDouble("Y_STEP");//5;
    public double PROXIMITY_DISTANCE_THRESHOLD = RunParams.getDouble("PROXIMITY_DISTANCE_THRESHOLD");
    public int MAX_CONNECTIONS = RunParams.getInt("MAX_CONNECTIONS");
    public String SAMPLING_METHOD = RunParams.get("SAMPLING_METHOD");

    ArrayList<Vertex> vertices;
    ArrayList<Edge> edges;

    public PRMGraph() {
        // Empty
    }

    /* Generates the road map using the provided map and vertex number. */
    public void generatePRM(PRMUtil util, OccupancyGrid map){
        if (SAMPLING_METHOD.equalsIgnoreCase("random")) {
            vertices = util.randomSample(map, NUMBER_OF_VERTICES);
        } else if (SAMPLING_METHOD.equalsIgnoreCase("grid")) {
            vertices = util.gridSample(map, X_STEP, Y_STEP);
        } else if (SAMPLING_METHOD.equalsIgnoreCase("cell")) {
            vertices = util.cellSample(map, CELL_WIDTH, TARGET_PER_CELL);
        } else {
            throw new IllegalStateException("Sampling method: " + SAMPLING_METHOD + " not known");
        }

        long start = System.currentTimeMillis();
        this.edges = util.connectVertices(vertices, PROXIMITY_DISTANCE_THRESHOLD, MAX_CONNECTIONS);
        System.out.println("All vertices connected in: "+(System.currentTimeMillis()-start)+" ms");
    }

    public boolean addVertex(Vertex v, PRMUtil util){
        if (vertices.contains(v)){
            // If we've added the vertex before.
            return false;
        }
        vertices.add(v);

        edges.addAll(util.connectVertexToGraph(v, vertices, PROXIMITY_DISTANCE_THRESHOLD, MAX_CONNECTIONS));

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
