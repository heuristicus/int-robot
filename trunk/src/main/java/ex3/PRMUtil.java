package ex3;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Random;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import org.ros.node.topic.Publisher;
import std_msgs.ColorRGBA;
import visualization_msgs.Marker;

public class PRMUtil {

    public static final int INFLATION_RADIUS = 5;

    public static final float MARKER_EDGE_WIDTH = 0.1f;
    public static final float MARKER_POINT_WIDTH = 0.2f;

    Random randGen;
    MessageFactory factory;
    OccupancyGrid inflatedMap;

    public PRMUtil(Random randGen, MessageFactory factory, OccupancyGrid inflatedMap){
        this.randGen = randGen;
        this.factory = factory;
        this.inflatedMap = inflatedMap;
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

        while (!foundOpen){
            randX = (randGen.nextFloat() * mapWidth);
            randY = (randGen.nextFloat() * mapHeight);

            foundOpen = checkPositionValidity((int) Math.round(randX), (int) Math.round(randY), mapWidth, mapHeight, buff, buffLength);

        }

        return new Vertex(randX * mapRes, randY * mapRes, factory);
    }

    /*
     * Checks whether the given pose is in free space on the given map.
     */
    public boolean checkPositionValidity(Pose p, OccupancyGrid map){
        int mapWidth = map.getInfo().getWidth();
        int mapHeight = map.getInfo().getHeight();
        float mapRes = map.getInfo().getResolution();
        return checkPositionValidity((int) Math.round(p.getPosition().getX() / mapRes),
                (int) Math.round(p.getPosition().getY() / mapRes),
                mapWidth,
                mapHeight,
                map.getData(),
                map.getData().capacity());
    }

    /*
     * Checks whether the position specified by the given x and y coordinates is
     * a valid position on the given map.
     */
    public boolean checkPositionValidity(int x, int y, int mapWidth, int mapHeight, ChannelBuffer buff, int buffLength){
        // get the index in the array for this random point
        int index = getMapIndex(x, y, mapWidth, mapHeight);
        if (index > 0 && index < buffLength) {
            Byte cell = buff.getByte(index);
            if (cell.byteValue() == 0) {
                // We are inside the map bounds and the cell is not occupied.
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }


    /*
     * Connects all vertices to all other vertices within a euclidean distance of
     * distanceThreshold. Also creates representative edges.
     */
    public ArrayList<Edge> connectVertices(ArrayList<Vertex> vertices, double distanceThreshold, int maxConnections){
        ArrayList<Edge> edges = new ArrayList<Edge>();
        ArrayList<Edge> temp = new ArrayList<Edge>();

        for (Vertex vertex : vertices) {
            temp = connectVertexToGraph(vertex, vertices, distanceThreshold, maxConnections);
            for (Edge edge : temp) {
                if (!edges.contains(edge)) {
                    edges.add(edge);
                }
            }
        }

        return edges;
    }

    /* Takes a list of vertices (a path) and removes intermediate points which
     * do not help us go around an obstacle. That is, if a is connected to b,
     * and b is connected to c, we check if there is a path from a to c and
     * if so, remove b from the path. We do this for the whole path until we
     * have as straight a path as possible */
    public ArrayList<Vertex> flattenDrunkenPath(ArrayList<Vertex> path) {
        ArrayList<Vertex> flatPath = (ArrayList) path.clone();
        boolean pathModified = true;
        while (pathModified) {
            pathModified = false;
            for (int i = 0; i < flatPath.size() - 2; i++) {
                // Check if current node can connect to the node after next
                Vertex a = flatPath.get(i);
                Vertex c = flatPath.get(i + 2);
                if (connectedInFreeSpace(inflatedMap, a, c)) {
                    flatPath.remove(i + 1); // Remove b
                    pathModified = true;
                }
            }
        }
        return flatPath;
    }

    /* Connect a vertex to other vertices in the graph. */
    public ArrayList<Edge> connectVertexToGraph(Vertex v, ArrayList<Vertex> vertices, double distanceThreshold, int maxConnections){
        ArrayList<Edge> edges = connectVertex_nearestN(v, vertices, maxConnections, maxConnections * 2);
        return edges;
    }

    /*
     * Connects a vertex to other vertices within distanceThreshold euclidean distance
     * of the specified vertex. Will not make more than maxConnections connections. The
     * vertex will not be connected to nodes that it is already connected to indirectly.
     * The node will be connected to nodes which are within the given threshold
     * based on their position in the array. The nodes with lower array indexes
     * are much more likely to be connected to.
     */
    public ArrayList<Edge> connectVertex_firstNInThreshold(Vertex v, ArrayList<Vertex> graph, double distanceThreshold, int maxConnections) {
	int connectedCount = 0;
        double distance = 0;
        ArrayList<Edge> connections = new ArrayList<Edge>();
        for (Vertex vert : graph) {
            if (vert.connectedVertices.size() >= maxConnections
                    || vert.getConnectedVertices().contains(v)){
                // If we've already been connected to this node, carry on.
                continue;
            }
            distance = getEuclideanDistance(v, vert);
            if (distance <= distanceThreshold && !isConnected(v, vert) && v != vert) {
                if (connectedInFreeSpace(inflatedMap,
                        v.getLocation().getX(),
                        v.getLocation().getY(),
                        vert.getLocation().getX(),
                        vert.getLocation().getY(), distance)) {

                    v.addConnectedVertex(vert);
                    vert.addConnectedVertex(v);
                    connections.add(new Edge(v, vert, distance));
                    connectedCount++;
                }
            }
	    if (connectedCount == maxConnections){
		break;
            }
        }

        return connections;
    }


    /*
     * Used to order vertices in a priority queue.
     */
    private class VertexTuple implements Comparable<VertexTuple> {
        public final Vertex v1;
        public final double distance;

        public VertexTuple(Vertex v1, double distance){
            this.v1 = v1;
            this.distance = distance;
        }

        /*
         * Compares two distances. Smaller distances are considered to be higher
         * priority than larger ones. If the distance in this object is smaller
         * than the one passed in, the value returned is +1.
         */
        @Override
        public int compareTo(VertexTuple o) {
            return Double.compare(distance, o.distance);
        }
    }

    /*
     * Attempts to connect the given node to the closest n nodes to it, starting
     * from the node that is closest, by checking distance to all vertices and
     * then attempting to make connections to the closest n of those.
     */
    public ArrayList<Edge> connectVertex_nearestN(Vertex v, ArrayList<Vertex> graph, int maxConnections, int maxAttempts) {
        PriorityQueue<VertexTuple> closestNodes = new PriorityQueue<VertexTuple>(graph.size());

        for (Vertex vert : graph) {
            if (vert == v || vert.connectedVertices.size() >= maxConnections){
                continue;
            }
            closestNodes.add(new VertexTuple(vert, getEuclideanDistance(v, vert)));
        }

        ArrayList<Edge> connections = new ArrayList<Edge>();

        int connectionAttempts = 0;
        int connectionCount = v.connectedVertices.size();
        while (connectionAttempts < maxAttempts && connectionCount < maxConnections) {
            VertexTuple vt = closestNodes.poll();
            // Run out of nodes to check so exit.
            if (vt == null){
                break;
            }
            Vertex vert = vt.v1;

            if (vert.getConnectedVertices().contains(v)){
                // If we've already been connected to this node, carry on.
                continue;
            }
            
            if (!isConnected(v, vert)) {
                if (connectedInFreeSpace(inflatedMap,
                        v.getLocation().getX(),
                        v.getLocation().getY(),
                        vert.getLocation().getX(),
                        vert.getLocation().getY(), vt.distance)) {

                    v.addConnectedVertex(vert);
                    vert.addConnectedVertex(v);
                    connections.add(new Edge(v, vert, vt.distance));
                    connectionCount++;
                }
            }
            connectionAttempts++;
        }
        
        return connections;
    }

    /* Checks if v1 is connected v2, either directly or indirectly. */
    public boolean isConnected(Vertex v1, Vertex v2){

        // If the node has no connections, then just return.
        if (v1.getConnectedVertices().isEmpty()){
            return false;
        }

	Queue<Vertex> toDo = new LinkedList<Vertex>();
	HashSet<Vertex> done = new HashSet<Vertex>();

	toDo.add(v1);

	while(! toDo.isEmpty()){
	    Vertex check = toDo.remove();
	    done.add(check);
	    for (Vertex connV : check.getConnectedVertices()){
		if (connV.equals(v2)){
		    return true;
		} else if (!done.contains(connV) && !toDo.contains(connV)){
		    toDo.add(connV);
		}
	    }
	}
        return false;
    }

    /* Gets the index of a specific point on the map. */
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

    Marker setUpMarker(String frameID, String namespace, int ID, int action, int type, ColorRGBA colour, Pose pose, Vector3 vector){
        Marker m = factory.newFromType(Marker._TYPE);
        setMarkerHeader(m, frameID, namespace, ID, action, type);

        if (pose != null) {
            m.setPose(pose);
        }

        if (vector != null){
            m.setScale(vector);
        }

        if (colour != null) {
            m.setColor(colour);
        }
        
        return m;
    }

    public Marker makePathMarker(List<Vertex> path, String namespace, String colour){
        Vector3 edgeVector = factory.newFromType(Vector3._TYPE);
        edgeVector.setX(MARKER_EDGE_WIDTH);

        Pose edgePose = factory.newFromType(Pose._TYPE);
        edgePose.getPosition().setX(0.0f);
        edgePose.getPosition().setY(0.0f);
        edgePose.getOrientation().setZ(1.0f);

        ColorRGBA edgeColour = factory.newFromType(ColorRGBA._TYPE);
        edgeColour.setA(1.0f);
        if (colour.equals("blue")){
            edgeColour.setB(1.0f);
        } else if (colour.equals("green")){
            edgeColour.setG(1.0f);
        } else {
            edgeColour.setR(1.0f);
        }


        Marker rtMarker = setUpMarker("/map", namespace, 2, Marker.ADD, Marker.LINE_STRIP, edgeColour, edgePose, edgeVector);

        for (Vertex vertex : path) {
//            System.out.println("Adding vertex " + vertex);
            Point tmp = factory.newFromType(Point._TYPE);
            tmp.setX(-vertex.getLocation().getX());
            tmp.setY(-vertex.getLocation().getY());
            rtMarker.getPoints().add(tmp);
        }

        return rtMarker;
    }

    List<Marker> getGraphMarkers(PRMGraph graph, OccupancyGrid map, String frameID) {
        Vector3 edgeVector = factory.newFromType(Vector3._TYPE);
        edgeVector.setX(MARKER_EDGE_WIDTH);

        Pose edgePose = factory.newFromType(Pose._TYPE);
        edgePose.getPosition().setX(0.0f);
        edgePose.getPosition().setY(0.0f);
        edgePose.getOrientation().setZ(1.0f);
        
        ColorRGBA edgeColour = factory.newFromType(ColorRGBA._TYPE);
        edgeColour.setA(0.5f);
        edgeColour.setR(1.0f);

        Vector3 pointVector = factory.newFromType(Vector3._TYPE);
        pointVector.setX(MARKER_POINT_WIDTH);
        pointVector.setY(MARKER_POINT_WIDTH);
        
        ColorRGBA pointColour = factory.newFromType(ColorRGBA._TYPE);
        pointColour.setA(0.5f);
        pointColour.setB(0.0f);
        pointColour.setG(1.0f);
        pointColour.setR(0.0f);

        Marker edgeMarker = setUpMarker(frameID, "edges", 0, Marker.ADD, Marker.LINE_LIST, edgeColour, edgePose, edgeVector);
        Marker pointMarker = setUpMarker(frameID, "points", 1, Marker.ADD, Marker.POINTS, pointColour, null, pointVector);

        for (Vertex v : graph.getVertices()) {
            // Add the vertices to the graph.
            pointMarker.getPoints().add(v.getLocation());
            for (Vertex connected : v.getConnectedVertices()) {
                /* Add a line from the current vertex to each connected vertex
                 * For some reason the coordinates must be reversed for this to
                 * display correctly in rviz. Probably some error with transforms
                 * or suchlike. */
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

    public double averageConnectionLength(PRMGraph graph){
        double sum = 0;

        for (Edge e : graph.getEdges()) {
            System.out.println(e);
            sum += e.edgeWeight();
        }

        return sum/graph.getEdges().size();
    }

    /*
     * Inflates the map so that obstacles are increased in size to the extent that
     * if the robot stays in the remaining free space, it will not collide with obstacles.
     * The actual obstacles are replaced with unknown space, and a certain area in the proxi-
     * mity of the obstacle is converted to obstacle space.
     */
    public static OccupancyGrid inflateMap(OccupancyGrid grid, Publisher<OccupancyGrid> pub) {
        /*
         * Copy data in the grid to a new channel buffer. We use this to check
         * what data was in the original buffer at each point.
         */
        ChannelBuffer original = ChannelBuffers.copiedBuffer(grid.getData());

        // Get an occupancy grid for us to put modified data into.
        OccupancyGrid inflatedMap = pub.newMessage();
        // Set the height, width and resolution to the same as that of the original.
        inflatedMap.setInfo(grid.getInfo());
        // Copy the data in the original buffer into the newly created grid.
        inflatedMap.setData(ChannelBuffers.copiedBuffer(grid.getData()));

        for (int i = 0; i < original.capacity(); i++) {
            // If data in the map indicates an obstacle, widen the obstacle by some amount
            if (original.getByte(i) == 100) {
                for (int yOffset = -INFLATION_RADIUS; yOffset <= INFLATION_RADIUS; yOffset++) {
                    int xOffset = grid.getInfo().getWidth() * yOffset;
                    //xRadius = RADIUS -
                    
                    for (int j = i + xOffset - INFLATION_RADIUS; j <= i + xOffset + INFLATION_RADIUS; j++) {
                        // If there is an obstacle very close to the zeroth index, avoid
                        // exceptions
                        if (j < 0) {
                            j = 0;
                        }
                        // Also avoid going over capacity
                        if (j >= original.capacity()) {
                            break;
                        }
                        // No point widening obstacles into unknown space or something
                        // which is already an obstacle
                        if (original.getByte(j) == -1 || original.getByte(j) == 100) {
                            continue;
                        }
                        // Set the byte to an obstacle in the inflated map
                        inflatedMap.getData().setByte(j, 100);
                    }
                }
                // Set the original obstacle position to unknown space just to
                // keep track of where it was before.
                inflatedMap.getData().setByte(i, -1);
            }
        }

        return inflatedMap;
    }

    public static boolean connectedInFreeSpace(OccupancyGrid map,
            Vertex a, Vertex b) {
        return connectedInFreeSpace(map,
                a.getLocation().getX(),
                a.getLocation().getY(),
                b.getLocation().getX(),
                b.getLocation().getY(),
                getEuclideanDistance(a, b));
    }

    /** Given a line (provided by two points), checks whether or not the line
     * intersects with obstacles on the map, returning true only if there is
     * a fully clear path between the two points (i.e. the line is in free space). */
    public static boolean connectedInFreeSpace(OccupancyGrid map, 
            double x1, double y1, double x2, double y2, double distance) {
        ChannelBuffer data = map.getData();
        int width = map.getInfo().getWidth();
        int height = map.getInfo().getHeight();
        float mapRes = map.getInfo().getResolution();

        x1 = x1 / mapRes;
        x2 = x2 / mapRes;
        y1 = y1 / mapRes;
        y2 = y2 / mapRes;
        distance = distance / mapRes;

        double yDiff = y2 - y1;
        double xDiff = x2 - x1;
        double xGradient = Math.abs(xDiff / distance);
        double yGradient = yDiff / distance;

        double x;
        double y;
        double bigX;
        if (x1 < x2) {
            x = x1;
            y = y1;
            bigX = x2;
        } else {
            x = x2;
            y = y2;
            bigX = x1;
            yGradient = -yGradient; // Invert gradient
        }

        boolean connectedFreely = true; // False when found obstacle

        //System.out.println("x1: " + x1 + " y1: " + y1 + " x2: " + x2 + " y2: " + y2);
        //System.out.println("xDiff: "+xDiff+" yDiff: "+yDiff+" distance: "+distance);
        //System.out.println("xGradient: "+xGradient+" yGradient: "+yGradient);
        while (x <= bigX && connectedFreely) {
            int index = getMapIndex((int)Math.round(x), (int)Math.round(y), (int) width, (int) height);

            if (index > 0 && index < data.capacity()) {
                //System.out.println("Index: "+index+" x: "+x+" y: "+y);

                // If we are on the map to begin with...
                Byte cellData = data.getByte(index);

                if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                    // If we're on the map, but the map has no data, or there is an obstacle...
                    connectedFreely = false;
                    break;
                }
            } else {
                System.out.println("ERROR! This shouldn't happen. ConnectedInFreeSpace "
                        + "called with out of bounds values. Do not ignore this MICHAL");
                System.out.println("Index: "+index+" x: "+x+" y: "+y);
                System.out.println("x1: "+x1+" y1: "+y1+" x2: "+x2+" y2: "+y2);
                connectedFreely = false;
                break;
            }

            x += xGradient;
            y += yGradient;
        }

        return connectedFreely;
    }

}
