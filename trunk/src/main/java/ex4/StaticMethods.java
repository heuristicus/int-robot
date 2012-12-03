/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import sensor_msgs.LaserScan;

/**
 *
 * @author robot
 */
public class StaticMethods {

    //the message factory to create ros objects
    public static MessageFactory messageFactory;

    /**
     * Saves the Laser readings to a CSV file
     * @param message The laser data
     * @throws IOException Exception creating/writing to file.
     */
    public static void saveDataToFile(LaserScan message) throws IOException {
        FileWriter fstream = new FileWriter("laserData.csv");
        BufferedWriter out = new BufferedWriter(fstream);
        float[] ranges = message.getRanges();
        for (int i = 0; i < ranges.length; i++) {
            out.write(ranges[i] + ",");
        }
        out.close();
    }

    /**
     * Converts a basic rotation about the z-axis (in radians) into the Quaternion
     * notation required by ROS transform and pose messages.
     * @param yaw rotate by this amount in radians
     * @param q Quaternion to be rotated
     * @return quaternion rotation about the z-axis (fine for robots which only
     * rotate using two-wheeled differential drive, for example)
     */
    public static Quaternion rotateQuaternion(Quaternion q_orig, double yaw) {
        // Create a temporary Quaternion toinitialPose.getOrientation( represent the change in heading
        Quaternion q_headingChange = createQuaternion();

        double p = 0;
        double y = yaw / 2.0;
        double r = 0;

        double sinp = Math.sin(p);
        double siny = Math.sin(y);
        double sinr = Math.sin(r);
        double cosp = Math.cos(p);
        double cosy = Math.cos(y);
        double cosr = Math.cos(r);

        q_headingChange.setX(sinr * cosp * cosy - cosr * sinp * siny);
        q_headingChange.setY(cosr * sinp * cosy + sinr * cosp * siny);
        q_headingChange.setZ(cosr * cosp * siny - sinr * sinp * cosy);
        q_headingChange.setW(cosr * cosp * cosy + sinr * sinp * siny);

        // Multiply new (heading-only) quaternion by the existing (pitch and bank) quaternion
        // Order is important! Original orientation is the second argument;
        // rotation which will be applied to the quaternion is the first argument.
        return multiply_quaternions(q_headingChange, q_orig);
    }

    /**
     * Multiplies two quaternions to give the rotation of qb by qa.
     * @param qa Quaternion rotation amount which will be applied to qb.
     * @param qb Quaternion to which rotation of qa will be applied.
     * @return Quaternion qb rotated by the amount represented by qa.
     */
    private static Quaternion multiply_quaternions(Quaternion qa, Quaternion qb) {
        Quaternion combined = createQuaternion();

        combined.setW(qa.getW() * qb.getW() - qa.getX() * qb.getX() - qa.getY() * qb.getY() - qa.getZ() * qb.getZ());
        combined.setX(qa.getX() * qb.getW() + qa.getW() * qb.getX() + qa.getY() * qb.getZ() - qa.getZ() * qb.getY());
        combined.setY(qa.getW() * qb.getY() - qa.getX() * qb.getZ() + qa.getY() * qb.getW() + qa.getZ() * qb.getX());
        combined.setZ(qa.getW() * qb.getZ() + qa.getX() * qb.getY() - qa.getY() * qb.getX() + qa.getZ() * qb.getW());

        return combined;
    }

    /**
     *
     * @return A blank identity quaternion (x,y,z = 0, w = 1) representing zero rotation.
     */
    public static Quaternion createQuaternion() {
        Quaternion q = messageFactory.newFromType(Quaternion._TYPE);

        // Set up 'identity' (blank) quaternion
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        return q;
    }

    /**
     *
     * @param q Quaternion describing a particular orientation about the z-axis
     * @return Equivalent orientation about the z-axis in radians
     */
    public static double getHeading(Quaternion q) {
        double w = q.getW();
        double x = q.getX();
        double y = q.getY();
        double z = q.getZ();

//        double pitch = Math.atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
//        double roll = Math.asin(-2*(x*z - w*y));
        double yaw = Math.atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);
        return yaw;
    }

    /**
     * Calculates the angle created, from 0 degrees, by two points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return The angle
     */
    public static double angleForTwoPoints(Point p1, Point p2) {
        return Math.atan2(
                p2.getY() - p1.getY(),
                p2.getX() - p1.getX());
    }

    /**
     * Calculates the euclidean distance between two points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return The euclidean distance
     */
    public static double distanceBetweenPoints(Point p1, Point p2) {
        double x = p1.getX() - p2.getX();
        double y = p1.getY() - p2.getY();
        return Math.sqrt((x * x) + (y * y));
    }

    /**
     * Creates a deep copy of a point
     * @param point The point to copy
     * @return The new, copied point
     */
    public static Point copyPoint(Point point) {
        Point pointCopy = messageFactory.newFromType(Point._TYPE);
        pointCopy.setX(point.getX());
        pointCopy.setY(point.getY());
        pointCopy.setZ(point.getZ());
        return pointCopy;
    }

    /**
     * Checks if the point is within the arraylist
     * @param points The list of points to check against
     * @param point The point to check for
     * @return if the point is in the list
     */
    public static boolean contains(ArrayList<Point> points, Point point) {
        for (Point p : points) {
            if (point.getX() == p.getX()
                    && point.getZ() == p.getZ()
                    && point.getY() == p.getY()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Creates a deep copy of a Pose
     * @param pose The Pose to copy
     * @return The new, copied Pose
     */
    public static Pose copyPose(Pose pose) {
        Pose poseCopy = messageFactory.newFromType(Pose._TYPE);
        poseCopy.getOrientation().setW(pose.getOrientation().getW());
        poseCopy.getOrientation().setX(pose.getOrientation().getX());
        poseCopy.getOrientation().setY(pose.getOrientation().getY());
        poseCopy.getOrientation().setZ(pose.getOrientation().getZ());
        poseCopy.getPosition().setX(pose.getPosition().getX());
        poseCopy.getPosition().setY(pose.getPosition().getY());
        poseCopy.getPosition().setZ(pose.getPosition().getZ());
        return poseCopy;
    }

    /***
     * checks whether the cell is occupied
     * @param x: the x position
     * @param y: the y position
     * @return: true if location is occupied
     */
    public static boolean cellIsOccupied(double x, double y, OccupancyGrid map) {
        ChannelBuffer data = map.getData();

        // Map data
        long map_width = map.getInfo().getWidth();
        long map_height = map.getInfo().getHeight();
        float map_resolution = map.getInfo().getResolution(); // in m per pixel


        // Particle position
        double x_orig = x / map_resolution;
        double y_orig = y / map_resolution;

        //check the indexes
        int index;
        if ((int) Math.round(x_orig) < 0 || (int) Math.round(y_orig) < 0 || x > (int) map_width || y > (int) map_height) {
            // If requested location is out of the bounds of the map
            index = -1;
        } else {
            index = ((int) Math.round(y_orig) * (int) map_width) + (int) Math.round(x_orig);
        }

        if (index > 0 && index < data.capacity()) {
            Byte cellData = data.getByte(index);

            if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Finds the index in the map date of a location on the map
     * @param x The x location on the map
     * @param y The y location on the map
     * @param width The width of the map
     * @param height The height of the map
     * @return An index within the map data
     */
    public static int getMapIndex(int x, int y, int width, int height) {
        // Given a map width and a location x,y, returns the index
        // of the linear map vector corresponding to this location
        // e.g. if x,y = 5,0 it should return the index 5 (6th element of the array)
        // or if x,y = 1,5 it should return the second of the first element of the 6th row, i.e. y * width + 1
        //            System.out.println("x: " + x + ",y: " + y + ", index: " + ((y*width) + x));
        if (x < 0 || y < 0 || x > width || y > height) {
            // If requested location is out of the bounds of the map
            return -1;
        } else {
            return (y * width) + x;
        }
    }
}
