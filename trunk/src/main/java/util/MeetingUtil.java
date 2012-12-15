package util;

import ex3.Vertex;
import ex4.Polygon2D;
import ex4.Printer;
import ex4.RectangleWithDepth;
import ex4.StaticMethods;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import java.util.ArrayList;
import java.util.HashSet;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import pf.AbstractLocaliser;

public class MeetingUtil {

    /*
     * Makes an exploration path out of a given set of vertices. The first point
     * on the path is the closest point to the starting position, and all subsequent
     * points on the path are the points which are in closest proximity to the
     * previous point.
     */
    public static ArrayList<Vertex> getExplorationPath(Pose startPose, ArrayList<Vertex> vertices) {
        ArrayList<Vertex> explorePath = new ArrayList<Vertex>(vertices.size());
        Point curPoint = startPose.getPosition();

        while (vertices.size() > 0) {
            int minIndex = -1;
            double minDist = Double.MAX_VALUE;
            for (int i = 0; i < vertices.size(); i++) {
                double thisDist = PRMUtil.getEuclideanDistance(curPoint, vertices.get(i).getLocation());
                if (thisDist < minDist) {
                    minIndex = i;
                    minDist = thisDist;
                }
            }
            Vertex curVertex = vertices.remove(minIndex);
            curPoint = curVertex.getLocation();
            explorePath.add(curVertex);
        }
        return explorePath;
    }

    /*
     * Checks whether a point is within the given set of meeting rooms.
     */
    public static boolean isPointInMeetingRooms(Polygon2D[] rooms, float x, float y){
        for (int j = 0; j < rooms.length; j++) {
            if (rooms[j].contains(x, y)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Removes exploration vertices which exist within the meeting room as meeting room does not need to be explored
     * counter used to tell us how many have been removed
     */
    public static void removeVerticesInMeetingRooms(ArrayList<Vertex> vertices, Polygon2D[] rooms) {
        int removedCounter = 0;
        for (int i = vertices.size() - 1; i >= 0; i--) {
            // if vertex is within meeting room, then remove the vertex so it is not explored
            Vertex nextVertex = vertices.get(i);
            for (int j = 0; j < rooms.length; j++) {
                if (rooms[j].contains(nextVertex.getLocation().getX(), nextVertex.getLocation().getY())) {
                    vertices.remove(i);
                    removedCounter++;
                    // A vertex can only be in a single meeting room (assuming
                    // non-overlapping rooms), so don't bother checking other ones
                    break;
                }
            }
        }
        Printer.println("removed " + removedCounter + " vertices found inside meeting rooms", "CYANF");
    }

    /*
     * Converts a float array received from the face_rects subscriber into an
     * array of rectangles with depth. The rectangle is represented by five
     * values; the x and y points of the top left corner, the width and height,
     * and the depth reading from the kinect.
     */
    public static RectangleWithDepth[] convert(float[] data) {
        int numberOfRectangles = (int) (data.length / 5.0);
        RectangleWithDepth[] result = new RectangleWithDepth[numberOfRectangles];
        for (int i = 0; i < numberOfRectangles; i++) {
            int index = i * 5;
            result[i] = new RectangleWithDepth(
                    data[index],
                    data[index + 1],
                    data[index + 2],
                    data[index + 3],
                    data[index + 4]);
        }
        return result;
    }

    /*
     * Get the centre of a rectangle as a point object.
     */
    public static Point getRectCentre(RectangleWithDepth rect, MessageFactory factory) {
        Point centre = factory.newFromType(Point._TYPE);
        centre.setX(rect.getCenterX());
        centre.setY(rect.getCenterY());
        return centre;
    }

     /*
     * Checks the disparity between two rectangles is below the value specified
     * in the parameter file.
     */
    public static boolean rectangleOverlapValid(RectangleWithDepth lastRect, RectangleWithDepth curRect, double maxCentreDisparity) {

        double xDisparity = Math.abs(curRect.getCenterX() - lastRect.getCenterX());
        double yDisparity = Math.abs(curRect.getCenterY() - lastRect.getCenterY());

        boolean xValid = xDisparity <= (maxCentreDisparity * curRect.width);
        boolean yValid = yDisparity <= (maxCentreDisparity * curRect.height);

        return xValid && yValid;
    }

    /*
     * Finds the rectangle in the array that is most similar to the rectangle
     * given.
     */
    public static RectangleWithDepth getMostSimilarRectangle(RectangleWithDepth[] rectangles,
            RectangleWithDepth testRect, double maxDepthDisparity, MessageFactory factory) {
        RectangleWithDepth mostSimilar = null;
        double similarDist = Double.MAX_VALUE;
        for (RectangleWithDepth rect : rectangles) {
            Point thisCentre = MeetingUtil.getRectCentre(rect, factory);
            Point testCentre = MeetingUtil.getRectCentre(testRect, factory);

            double msgDepth = rect.getDepth();
            double testDepth = testRect.getDepth();
            double thisDist = PRMUtil.getEuclideanDistance(testCentre, thisCentre);

            if (thisDist < similarDist && Math.abs(msgDepth - testDepth) <= maxDepthDisparity) {
                mostSimilar = rect;
                similarDist = thisDist;
            }
        }
        return mostSimilar;
    }


    /*
     * Go through an array to find which rectangle in it has the smallest depth
     * value.
     */
    public static RectangleWithDepth getClosestRectangle(RectangleWithDepth[] rectangles) {
        RectangleWithDepth closestRect = null;
        for (RectangleWithDepth rect : rectangles) {
            if (closestRect == null || rect.getDepth() < closestRect.getDepth()) {
                closestRect = rect;
            }
        }
        return closestRect;
    }

    /*
     * Checks whether the centre point of curRect is within the rectangle lastRect.
     */
    public static boolean checkPointCentreInRectangle(RectangleWithDepth lastRect, RectangleWithDepth curRect) {
        return lastRect.contains(curRect.getCenterX(), curRect.getCenterY());
    }

    /*
     * Gets a polygon representing the field of view of a camera or somesuch
     * device. The field is assumed to be cone-shaped, and this shape is approximated.
     * The minDistance parameter represents the minimum distance at which the
     * device can perform its required task.
     */
    public static Polygon2D getFOVpoly(Pose pose, double minDistance, double maxDistance, double fovAngle, double mapRes) {
        double heading = StaticMethods.getHeading(pose.getOrientation());
        double halfAngle = Math.toRadians(fovAngle / 2.0);
        Polygon2D triangle = new Polygon2D();

        double scaledX, scaledY;
//        scaledX = pose.getPosition().getX() / mapRes;
//        scaledY = pose.getPosition().getY() / mapRes;
//        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (minDistance * Math.cos(heading + halfAngle))) / mapRes;
        scaledY = (pose.getPosition().getY() + (minDistance * Math.sin(heading + halfAngle))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (minDistance * Math.cos(heading - halfAngle))) / mapRes;
        scaledY = (pose.getPosition().getY() + (minDistance * Math.sin(heading - halfAngle))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (maxDistance * Math.cos(heading - halfAngle))) / mapRes;
        scaledY = (pose.getPosition().getY() + (maxDistance * Math.sin(heading - halfAngle))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (maxDistance * Math.cos(heading - halfAngle / 2))) / mapRes;
        scaledY = (pose.getPosition().getY() + (maxDistance * Math.sin(heading - halfAngle / 2))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (maxDistance * Math.cos(heading + halfAngle / 2))) / mapRes;
        scaledY = (pose.getPosition().getY() + (maxDistance * Math.sin(heading + halfAngle / 2))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        scaledX = (pose.getPosition().getX() + (maxDistance * Math.cos(heading + halfAngle))) / mapRes;
        scaledY = (pose.getPosition().getY() + (maxDistance * Math.sin(heading + halfAngle))) / mapRes;
        triangle.addPoint(scaledX, -scaledY);

        return triangle;
    }

    /*
     * Normalises a heatmap so that the values inside it are within a predefined
     * range.
     */
    public static void normaliseHeatMap(OccupancyGrid heatMapGrid, double[] heatDataArray) {
        double maxHeatValue = 0;
        for (double heatValue : heatDataArray) {
            if (heatValue > maxHeatValue) {
                maxHeatValue = heatValue;
            }
        }

        for (int i = 0; i < heatMapGrid.getData().array().length; i++) {
            int value =  (int)((heatDataArray[i] / maxHeatValue) * 100.0);
            heatMapGrid.getData().array()[i] = (byte) value;
        }
    }

    /*
     * Updates data inside a heatmap given list of lists of integers which come
     * from raytraced FOV lines.
     */
    public static void updateHeatDataRay(ArrayList<ArrayList<Integer>> coveredIndices, double[] heatData){
        for(ArrayList<Integer> ray : coveredIndices){
            updateHeatData(ray, heatData);
        }
    }

    public static void updateHeatData(ArrayList<Integer> coveredIndices, double[] heatData){
        for (Integer i : coveredIndices) {
            heatData[i]++;
        }
    }

    /*
     * Calculates the position of a detected visual feature which is directly
     * in front of the robot.
     */
    public static PoseStamped getObjectLocation(Pose estimatedPoseCopy, double depth, MessageFactory factory) {
        PoseStamped personLocation = factory.newFromType(PoseStamped._TYPE);
        double heading = StaticMethods.getHeading(estimatedPoseCopy.getOrientation());
        personLocation.getPose().getPosition().setX(
                estimatedPoseCopy.getPosition().getX()
                + (depth * Math.cos(heading)));
        personLocation.getPose().getPosition().setY(
                estimatedPoseCopy.getPosition().getY()
                + (depth * Math.sin(heading)));
        return personLocation;
    }

    /*
     * Gets a pose from stage's base_pose_ground_truth topic and normalises it to
     * match with the expected position on the rviz map. For some unknown reason
     * it is necessary to do this. The following operations are performed:
     * x = -y - 0.6
     * y = x - 0.6
     * theta = -theta
     */
    public static Pose normaliseStagePose(Pose stagePose, MessageFactory factory){
        Pose norm = factory.newFromType(Pose._TYPE);
        norm.getPosition().setX(-stagePose.getPosition().getY() - 0.6);
        norm.getPosition().setY(stagePose.getPosition().getX() - 0.6);
        AbstractLocaliser.rotateQuaternion(AbstractLocaliser.createQuaternion(), -AbstractLocaliser.getHeading(stagePose.getOrientation()));
        return norm;
    }

    /*
     * fov_angle is the kinect vision fov in degrees, heading is the robot heading
     * in radians. range_min is the minimum range at which things can be detected
     * range_max is the maximum detection range. The projected fov will be drawn
     * onto the map provided. An arraylist of integers is returned which contains
     * the indices that each ray traced went through. Will not place any points
     * within the rooms in the polygon array provided.
     */
    public static ArrayList<ArrayList<Integer>> projectFOV(double x, double y, double bearing, int fovAngle,
            double minRange, double maxRange, double angleStep, OccupancyGrid map, OccupancyGrid mapToModify,
            Polygon2D[] rooms) {
        ChannelBuffer data = map.getData();
        ChannelBuffer modData = mapToModify.getData();
        ArrayList<ArrayList<Integer>> fovRays = new ArrayList<ArrayList<Integer>>();
        // As the ROS angle system has 0 deg = east and increases anti-clockwise
        // but the Quaternion trigonometry and particle raytracing assume 0 deg = north / clockwise,
        // we must first convert to 0 deg = north / clockwise by subtracting PI/2
//        bearing -= Math.PI / 2;
        bearing *= -1;
        double freeBefore = countFreePixels(mapToModify, rooms);

        double fovRad = Math.toRadians(fovAngle);

        double firstRayAngle = GeneralUtil.normaliseAngle(bearing - fovRad / 2);
        double lastRayAngle = GeneralUtil.normaliseAngle(bearing + fovRad / 2);
        double currentRayAngle = firstRayAngle;
        double rayAngleIncrement = Math.toRadians(angleStep) * GeneralUtil.angleDirection(firstRayAngle, lastRayAngle);

//        System.out.println("Start: " + Math.toDegrees(firstRayAngle) + " End: " + Math.toDegrees(lastRayAngle));
        // Map data
        long mapWidth = map.getInfo().getWidth();
        long mapHeight = map.getInfo().getHeight();
        float mapRes = map.getInfo().getResolution(); // in m per pixel

        for (int i = 0; i < fovAngle / angleStep; i++) {
            ArrayList<Integer> ray = new ArrayList<Integer>();
            // Find gradient of the line of sight in x,y plane, assuming 0 deg = north
            double gradX = Math.sin(currentRayAngle);
            double gradY = Math.cos(currentRayAngle);

//            double grad_x = Math.sin(Math.toRadians(i));
//            double grad_y = Math.cos(Math.toRadians(i));

            // Particle position
            double startX = x / mapRes;
            double startY = y / mapRes;

            // Min and max range positions relative to the current position
            double maxX = maxRange * gradX / mapRes;
            double maxY = maxRange * gradY / mapRes;

            double minX = minRange * gradX / mapRes;
            double minY = minRange * gradY / mapRes;

            double curX = startX;
            double curY = startY;
            boolean occupied = false; // Have we found an occupied cell yet?

            // Stop travelling away from the robot when we reach max range of
            // laser or an occupied cell
            while (Math.abs(curX - startX) < Math.abs(maxX)
                    && Math.abs(curY - startY) < Math.abs(maxY)
                    && !occupied) {
                curX += gradX;
                curY += gradY;

                int index = GeneralUtil.getMapIndex((int) Math.round(curX), (int) Math.round(curY), (int) mapWidth, (int) mapHeight);

                if (index > 0 && index < data.capacity()) {
                    // If we are on the map to begin with...
                    Byte cellData = data.getByte(index);

                    if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                        // If we're on the map, but the map has no data, or there is an obstacle...
                        occupied = true;
                    } else {
                        if (Math.abs(curX - startX) < Math.abs(minX) && Math.abs(curY - startY) < Math.abs(minY) || isPointInMeetingRooms(rooms, (float)curX * mapRes, (float)curY * mapRes)) {
                            // Don't draw anything if the current range is smaller than the
                            // minimum range.
                            continue;
                        } else {
                            occupied = false;
                            ray.add(index);
                            modData.setByte(index, 100);
                        }
                    }
                } else {
                    occupied = true;
                }
            }
            currentRayAngle = GeneralUtil.normaliseAngle(currentRayAngle + rayAngleIncrement);
            fovRays.add(ray);
        }

        double freeAfter = countFreePixels(mapToModify, rooms);

        Printer.println("coverage: Free before: " + freeBefore + ", free after: " + freeAfter + ", Pose: " + x + ", " + y + ", " + bearing);
        return fovRays;
    }


    /*
     * Same as the projectFOV function, but does not return the indices that each
     * ray traces through. Instead, returns a single array of indices with no
     * duplicates for all of the traced rays.
     */
    public static ArrayList<Integer> simple_projectFOV(double x, double y, double bearing, int fovAngle,
            double minRange, double maxRange, double angleStep, OccupancyGrid map, OccupancyGrid mapToModify,
            Polygon2D[] rooms) {
        ChannelBuffer data = map.getData();
        ChannelBuffer modData = mapToModify.getData();
        ArrayList<Integer> traceIndices = new ArrayList<Integer>();
        // Store indices in a hashset so we can check if they've been seen before
        // more efficiently.
        HashSet<Integer> traced = new HashSet<Integer>();
        // As the ROS angle system has 0 deg = east and increases anti-clockwise
        // but the Quaternion trigonometry and particle raytracing assume 0 deg = north / clockwise,
        // we must first convert to 0 deg = north / clockwise by subtracting PI/2
//        bearing -= Math.PI / 2;
        bearing *= -1;
        double freeBefore = countFreePixels(mapToModify, rooms);

        double fovRad = Math.toRadians(fovAngle);

        double firstRayAngle = GeneralUtil.normaliseAngle(bearing - fovRad / 2);
        double lastRayAngle = GeneralUtil.normaliseAngle(bearing + fovRad / 2);
        double currentRayAngle = firstRayAngle;
        double rayAngleIncrement = Math.toRadians(angleStep) * GeneralUtil.angleDirection(firstRayAngle, lastRayAngle);

//        System.out.println("Start: " + Math.toDegrees(firstRayAngle) + " End: " + Math.toDegrees(lastRayAngle));
        // Map data
        long mapWidth = map.getInfo().getWidth();
        long mapHeight = map.getInfo().getHeight();
        float mapRes = map.getInfo().getResolution(); // in m per pixel

        for (int i = 0; i < fovAngle / angleStep; i++) {
            // Find gradient of the line of sight in x,y plane, assuming 0 deg = north
            double gradX = Math.sin(currentRayAngle);
            double gradY = Math.cos(currentRayAngle);

//            double grad_x = Math.sin(Math.toRadians(i));
//            double grad_y = Math.cos(Math.toRadians(i));

            // Particle position
            double startX = x / mapRes;
            double startY = y / mapRes;

            // Min and max range positions relative to the current position
            double maxX = maxRange * gradX / mapRes;
            double maxY = maxRange * gradY / mapRes;

            double minX = minRange * gradX / mapRes;
            double minY = minRange * gradY / mapRes;

            double curX = startX;
            double curY = startY;
            boolean occupied = false; // Have we found an occupied cell yet?

            // Stop travelling away from the robot when we reach max range of
            // laser or an occupied cell
            while (Math.abs(curX - startX) < Math.abs(maxX)
                    && Math.abs(curY - startY) < Math.abs(maxY)
                    && !occupied) {
                curX += gradX;
                curY += gradY;

                int index = GeneralUtil.getMapIndex((int) Math.round(curX), (int) Math.round(curY), (int) mapWidth, (int) mapHeight);

                if (index > 0 && index < data.capacity()) {
                    // If we are on the map to begin with...
                    Byte cellData = data.getByte(index);

                    if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                        // If we're on the map, but the map has no data, or there is an obstacle...
                        occupied = true;
                    } else {
                        if (Math.abs(curX - startX) < Math.abs(minX) && Math.abs(curY - startY) < Math.abs(minY) || isPointInMeetingRooms(rooms, (float)curX * mapRes, (float)curY * mapRes)) {
                            // Don't draw anything if the current range is smaller than the
                            // minimum range.
                            continue;
                        } else {
                            occupied = false;
                            // We only add to the array if the index has not been
                            // seen before.
                            if (!traced.contains(index)){
                                traceIndices.add(index);
                                traced.add(index);
                                modData.setByte(index, 100);
                            }
                        }
                    }
                } else {
                    occupied = true;
                }
            }
            currentRayAngle = GeneralUtil.normaliseAngle(currentRayAngle + rayAngleIncrement);
        }

        double freeAfter = countFreePixels(mapToModify, rooms);

        Printer.println("coverage: Free before: " + freeBefore + ", free after: " + freeAfter + ", Pose: " + x + ", " + y + ", " + bearing);
        return traceIndices;
    }

    /*
     * Calculates how many free pixels there are in the given map.
     */
    public static double countFreePixels(OccupancyGrid map, Polygon2D[] rooms){
        final int mapHeight = map.getInfo().getHeight();
        final int mapWidth = map.getInfo().getWidth();
        final float mapRes = map.getInfo().getResolution();
        int index;
        int pixel;
        int freePixels = 0;
        
        for (int y = 0; y < mapHeight; y++) {
            for (int x = 0; x < mapWidth; x++) {
                index = GeneralUtil.getMapIndex(y, x, mapWidth, mapHeight); // X and y flipped. Go figure
                pixel = map.getData().getByte(index);
                if (pixel != 100 && pixel != -1) {
                    if (!isPointInMeetingRooms(rooms, y * mapRes, x * mapRes)) { // X and y flipped. Go figure
                        freePixels++;
                    }
                }
            }
        }
        return freePixels;
    }

}
