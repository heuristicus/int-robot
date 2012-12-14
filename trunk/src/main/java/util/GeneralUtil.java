package util;

import ex3.PRMGraph;
import ex3.Vertex;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.Vector3;
import java.util.ArrayList;
import java.util.List;
import launcher.RunParams;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import std_msgs.ColorRGBA;
import visualization_msgs.Marker;

public class GeneralUtil {

    public static final float MARKER_EDGE_WIDTH = RunParams.getFloat("MARKER_EDGE_WIDTH");
    public static final float MARKER_POINT_WIDTH = RunParams.getFloat("MARKER_POINT_WIDTH");

    MessageFactory factory;

    public GeneralUtil(MessageFactory factory){
        this.factory = factory;
    }

    /* Gets the index of a specific point on the map. */
    public static int getMapIndex(int x, int y, int width, int height) {
        if (x < 0 || y < 0 || x > width || y > height) {
            // If requested location is out of the bounds of the map
            return -1;
        } else {
            return (y * width) + x;
        }
    }

    /*
     * Converts an arraylist of vertices to a pose array
     */
    public PoseArray convertVertexList(ArrayList<Vertex> varr) {
        PoseArray pose = factory.newFromType(PoseArray._TYPE);

        ArrayList<Pose> arr = new ArrayList<Pose>();

        for (Vertex v : varr) {
            arr.add(v.makePose(factory));
        }

        pose.setPoses(arr);

        return pose;
    }

    /*
     * Sets the header of a specific marker to the given values.
     */
    public static void setMarkerHeader(Marker m, String frameID, String namespace, int ID, int action, int type) {
        m.getHeader().setFrameId(frameID);
        m.setNs(namespace);
        m.setAction(action);
        m.setId(ID);
        m.setType(type);
    }

    public Marker setUpMarker(String frameID, String namespace, int ID, int action,
            int type, ColorRGBA colour, Pose pose, Vector3 vector) {
        return setUpMarker(frameID, namespace, ID, action, type, colour, pose, vector, factory);
    }
    /*
     * Creates a marker message using the given values
     */

    public static Marker setUpMarker(String frameID, String namespace, int ID, int action,
            int type, ColorRGBA colour, Pose pose, Vector3 vector, MessageFactory messageFactory) {
        Marker m = messageFactory.newFromType(Marker._TYPE);
        setMarkerHeader(m, frameID, namespace, ID, action, type);

        if (pose != null) {
            m.setPose(pose);
        }

        if (vector != null) {
            m.setScale(vector);
        }

        if (colour != null) {
            m.setColor(colour);
        }

        return m;
    }

    /*
     * Makes a list of markers out of a path list, so that the path can be displayed
     * in rviz.
     */
    public Marker makePathMarker(List<Vertex> path, String namespace, String colour, int frameID) {
        Vector3 edgeVector = factory.newFromType(Vector3._TYPE);
        edgeVector.setX(MARKER_EDGE_WIDTH);

        Pose edgePose = factory.newFromType(Pose._TYPE);
        edgePose.getPosition().setX(0.0f);
        edgePose.getPosition().setY(0.0f);
        edgePose.getOrientation().setZ(1.0f);

        ColorRGBA edgeColour = factory.newFromType(ColorRGBA._TYPE);
        edgeColour.setA(1.0f);
        if ("blue".equals(colour)) {
            edgeColour.setB(1.0f);
        } else if ("green".equals(colour)) {
            edgeColour.setG(1.0f);
        } else if ("orange".equals(colour)) {
            edgeColour.setR(1.0f);
            edgeColour.setG(0.8f); // I'm sorry
        } else {
            edgeColour.setR(1.0f);
        }


        Marker rtMarker = setUpMarker("/map", namespace, frameID, Marker.ADD, Marker.LINE_LIST, edgeColour, edgePose, edgeVector);

        for (int i = 0; i < path.size() - 1; i++) {
//            System.out.println("Adding vertex " + vertex);
            Point tmp = factory.newFromType(Point._TYPE);
            tmp.setX(-path.get(i).getLocation().getX());
            tmp.setY(-path.get(i).getLocation().getY());

            Point tmp2 = factory.newFromType(Point._TYPE);
            tmp2.setX(-path.get(i + 1).getLocation().getX());
            tmp2.setY(-path.get(i + 1).getLocation().getY());

            rtMarker.getPoints().add(tmp);
            rtMarker.getPoints().add(tmp2);
        }

        return rtMarker;
    }

    /*
     * Makes the graph into a set of markers so that it can be displayed in rviz
     */
    public List<Marker> getGraphMarkers(PRMGraph graph, OccupancyGrid map, String frameID) {
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

        ArrayList<Vertex> vertices = graph.getVertices();
        synchronized (vertices) {
            for (Vertex v : vertices) {
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
        }
        List<Marker> mList = new ArrayList<Marker>();

        mList.add(edgeMarker);
        mList.add(pointMarker);

        return mList;
    }

    public static OccupancyGrid copyMap(OccupancyGrid origMap, MessageFactory factory) {
        // Copy data in the grid to a new channel buffer
        ChannelBuffer original = ChannelBuffers.copiedBuffer(origMap.getData());

        // Get an occupancy grid for us to put modified data into.
        OccupancyGrid newMap = factory.newFromType(OccupancyGrid._TYPE);

        // Copy the original buffer into the newly created grid.
        newMap.setInfo(origMap.getInfo());
        newMap.setData(ChannelBuffers.copiedBuffer(origMap.getData()));

        return newMap;
    }

    

    /*
     * Modifies the angle given (in radians) to make it correspond with the required
     * range of angles, -pi to pi, where -ve rotation is clockwise.
     */
    public static double normaliseAngle(double angleRad) {
        double res;
        if (angleRad < -Math.PI) {
            System.out.println("Normalising " + angleRad + " (" + Math.toDegrees(angleRad) + ")");
            res = Math.PI + (angleRad % -Math.PI);
            System.out.println("normalised to:" + Math.toDegrees(res) + "pi%theta: " + (Math.PI % angleRad));
        } else if (angleRad > Math.PI) {
            System.out.println("Normalising " + angleRad + " (" + Math.toDegrees(angleRad) + ")");
            res = -Math.PI + (angleRad % Math.PI);
            System.out.println("normalised to:" + Math.toDegrees(res) + "pi%theta: " + (-Math.PI % angleRad));
        } else {
            res = angleRad;
        }

        return res;
    }

    /*
     * Calculate sign necessary to rotate from firstangle towards
     * lastangle by the shortest rotation. i.e. if you are at -179 and want to
     * rotate to 179, in which direction should you turn. (In this case, the
     * answer is in the -ve direction.
     */
    public static double angleDirection(double firstAngle, double lastAngle) {
        int sign;
        if (lastAngle < 0 && firstAngle > 0) {
            if (firstAngle >= Math.PI / 2) {
                sign = +1;
            } else {
                sign = -1;
            }
        } else if (firstAngle < 0 && lastAngle > 0) {
            if (lastAngle >= Math.PI / 2) {
                sign = -1;
            } else {
                sign = +1;
            }
        } else {
            sign = +1;
        }
        return sign;
    }
}