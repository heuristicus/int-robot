package ex3;

import java.util.ArrayList;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;

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

}
