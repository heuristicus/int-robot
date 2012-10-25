package ex3;

import geometry_msgs.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import visualization_msgs.Marker;

public class PRMUtil {

    Random randGen;
    MessageFactory factory;

    public PRMUtil(Random randGen, MessageFactory factory){
        this.randGen = randGen;
        this.factory = factory;
    }

    /*
     * Gets the euclidean distance between two points
     */
    public static double getEuclideanDistance(Vertex v1, Vertex v2){
        return Math.sqrt(Math.pow(v1.getLocation().getX() - v2.getLocation().getX(), 2) + Math.pow(v1.getLocation().getY() - v2.getLocation().getY(), 2));
    }

    /*
     * Calculates the weight of an edge.
     */
    public static double getEdgeWeight(Edge e){
        return getEuclideanDistance(e.a, e.b);
    }

    /*
     * Generates a set of random vertices which are guaranteed to be in free space
     * on the map. This version of the method does not take into account the size
     * or orientation of the robot.
     */
    public ArrayList<Vertex> generateRandomVertices(OccupancyGrid map, int vertexNum){
        final ChannelBuffer buff = map.getData();

        ArrayList<Vertex> randomVertices = new ArrayList<Vertex>();

        final int mapHeight = map.getInfo().getHeight();
        final int mapWidth = map.getInfo().getWidth();
        final float mapRes = map.getInfo().getResolution();

        for (int i = 0; i < vertexNum; i++) {
            randomVertices.add(getRandomVertex(mapWidth, mapHeight, mapRes, buff));
        }

        return randomVertices;
    }

    /*
     * Generates random points on the map until one that is valid is found.
     */
    public Vertex getRandomVertex(int mapWidth, int mapHeight, float mapRes, ChannelBuffer buff){
        boolean foundOpen = false;
        float randX = 0;
        float randY = 0;
        final int buffLength = buff.capacity();
        int index;

        while (!foundOpen){
            randX = (randGen.nextFloat() * mapWidth);
            randY = (randGen.nextFloat() * mapHeight);

            // get the index in the array for this random point
            index = getMapIndex((int) Math.round(randX), (int) Math.round(randY), mapWidth, mapHeight);
            if (index > 0 && index < buffLength){
                Byte cell = buff.getByte(index);
                if (cell.byteValue() == 0) {
                    // We are inside the map bounds and the cell is not occupied.
                    foundOpen = true;
                }
            }
        }

        return new Vertex(randX * mapRes, randY * mapRes, factory);
    }

    /*
     * Connects all vertices to all other vertices within a euclidean distance of
     * distanceThreshold.
     */
    public void connectVertices(ArrayList<Vertex> vertices, double distanceThreshold){
        for (Vertex vertex : vertices) {
            connectVertex(vertex, vertices, distanceThreshold);
        }
    }

    /*
     * Connects a vertex to other vertices within distanceThreshold euclidean distance
     * of the specified vertex.
     */
    public void connectVertex(Vertex v, ArrayList<Vertex> graph, double distanceThreshold) {
        for (Vertex vert : graph) {
            if (getEuclideanDistance(v, vert) <= distanceThreshold) {
                v.addConnectedVertex(vert);
            }
        }
    }

    /*
     * Gets the index of a specific point on the map.
     */
    public static int getMapIndex(int x, int y, int width, int height){
        if (x < 0 || y < 0 || x > width || y > height) {
            // If requested location is out of the bounds of the map
            return -1;
        } else {
            return (y * width) + x;
        }
    }

    void setMarkerHeader(Marker m, String frameID, String namespace, int ID, int action, int type){
        m.getHeader().setFrameId(frameID);
        m.setNs(namespace);
        m.setAction(action);
        m.setId(ID);
        m.setType(type);
    }

    Marker setUpEdgeMarker(String frameID, String namespace, int ID, int action, int type){
        Marker m = factory.newFromType(Marker._TYPE);
        setMarkerHeader(m, frameID, "edges", 0, action, type);
        
        m.getPose().getPosition().setX(0.0f);
        m.getPose().getPosition().setY(0.0f);
        m.getPose().getOrientation().setZ(1.0f);
        m.getScale().setX(0.1f);
        m.getColor().setA(1.0f);
        m.getColor().setR(1.0f);

        return m;
    }

    Marker setUpPointMarker(String frameID, String namespace, int ID, int action, int type) {
        Marker m = factory.newFromType(Marker._TYPE);
        setMarkerHeader(m, frameID, "points", 1, action, type);

        m.getScale().setX(0.2f);
        m.getScale().setY(0.2f);
        m.getColor().setA(1.0f);
        m.getColor().setB(1.0f);
        m.getColor().setG(1.0f);
        m.getColor().setR(1.0f);

        return m;
    }

    List<Marker> getGraphMarkers(PRMGraph graph, String frameID) {
        Marker edgeMarker = setUpEdgeMarker(frameID, "edges", 0, Marker.ADD, Marker.LINE_LIST);
        Marker pointMarker = setUpPointMarker(frameID, "edges", 0, Marker.ADD, Marker.POINTS);

        for (Vertex v : graph.getVertices()) {
            // Add the vertices to the graph.
            pointMarker.getPoints().add(v.getLocation());
            for (Vertex connected : v.getConnectedVertices()) {
                /*
                 * Add a line from the current vertex to each connected vertex
                 * For some reason the coordinates must be reversed for this to
                 * display correctly in rviz. Probably some error with transforms
                 * or suchlike.
                 */
                Point cur = factory.newFromType(Point._TYPE);
                cur.setX(-v.getLocation().getX());
                cur.setY(-v.getLocation().getY());
                Point link = factory.newFromType(Point._TYPE);
                link.setX(-connected.getLocation().getX());
                link.setY(-connected.getLocation().getY());
                edgeMarker.getPoints().add(cur);
                edgeMarker.getPoints().add(link);
            }
        }

        List<Marker> mList = new ArrayList<Marker>();

        mList.add(edgeMarker);
        mList.add(pointMarker);

        return mList;
    }

    Marker getPointMarker(PRMGraph graph, MessageFactory factory, String frameID) {
        Marker m = factory.newFromType(Marker._TYPE);

        setMarkerHeader(m, frameID, "points", 1, m.ADD, m.POINTS);

        m.getPose().getOrientation().setZ(1.0f);
        m.getScale().setX(0.1f);
        m.getColor().setA(1.0f);
        m.getColor().setR(1.0f);

        for (Vertex v : graph.getVertices()) {
            m.getPoints().add(v.getLocation());
        }
        
        return m;
    }

    
}
