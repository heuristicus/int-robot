package pf;

/**
 *
 * @author rowanms
 * An abstract Localiser which needs to be extended as PFLocaliser before PFLocalisationNode will work.
 * 
 */
import java.util.Random;

import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import geometry_msgs.Pose;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.PoseArray;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import tf.tfMessage;
import nav_msgs.OccupancyGrid;
import sensor_msgs.LaserScan;

public abstract class AbstractLocaliser extends PFLocalisationNode {

    // Data fields
    private PoseWithCovarianceStamped estimatedpose;
    private OccupancyGrid map;
    private PoseArray particlecloud;
    private tfMessage tf;
    
    private static final double INIT_X = 10; // Initial x location of robot (metres)
    private static final double INIT_Y = 5; // Initial y location of robot (metres)
    private static final double INIT_Z = 0; // Initial z location of robot (metres)
    private static final double INIT_HEADING = 0; // Initial orientation of robot (radians)
    private static final double ROTATION_NOISE = Math.PI/30; // Add noise to each odometry rotation reading
    private static final double POSITION_NOISE = 0.15; // Add noise to each odometry position reading 
    
    private static final double PIOVERTWO = Math.PI/2; // For faster calculations
    
    private double prev_odom_x; // Previous odometry translation from origin
    private double prev_odom_y; // Previous odometry translation from origin
    private double prev_odom_heading; // Previous heading from odometry data
    private boolean odom_initialised = false; // Request robot's initial odometry values to be recorded in prev_odom
    
    protected static MessageFactory messageFactory; // MessageFactory for creating Messages without a Node
    
    public AbstractLocaliser() {    
        
        // Call superclass constructor
        super();
        
        // Create MessageFactory
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
        messageFactory = nodeConfiguration.getTopicMessageFactory();
        
        // Initialise fields
        this.estimatedpose = messageFactory.newFromType(PoseWithCovarianceStamped._TYPE);
        this.map = messageFactory.newFromType(OccupancyGrid._TYPE);
        this.particlecloud = messageFactory.newFromType(PoseArray._TYPE);
        this.tf = messageFactory.newFromType(tfMessage._TYPE);
        
        // Set 'previous' translation to origin
        // All Transform translations are given relative to 0,0,0, not in absolute coords.
        this.prev_odom_x = 0;
        this.prev_odom_y = 0;
        this.prev_odom_heading = 0;

        // Set default initial pose to initial position and orientation.
        estimatedpose.getPose().getPose().getPosition().setX(INIT_X);
        estimatedpose.getPose().getPose().getPosition().setX(INIT_Y);
        estimatedpose.getPose().getPose().getPosition().setX(INIT_Z);
        estimatedpose.getPose().setCovariance(getEmptyCov()); // Currently not making use of covariance matrix
        estimatedpose.getPose().getPose().setOrientation(rotateQuaternion(createQuaternion(), INIT_HEADING));
    }
    
    /**
     * Constructs an all-zero covariance matrix for the location estimate
     * @return 36-element covariance matrix (currently all zero)
     */
    private double[] getEmptyCov() {
        double[] cov = new double[36]; // 36 comes from the msg definition, and is fixed
        for (int i = 0; i < cov.length; i++) {
            cov[i] = 0.0;
        }
        return cov;
    }


    /**
     * Called whenever an initialpose message is received (to change the starting
     * location of the robot), or a new map is received.
     * @param initialpose the initial pose estimate
     * @return PoseArray object containing ArrayList of Poses,
     */
    public abstract PoseArray initialisePF(PoseWithCovarianceStamped initialpose);


    /**
     * updatePF is called whenever there is a new LaserScan message.
     * This calls update methods (implemented by subclass) to do actual particle
     * filtering, given the map and the LaserScan, and then updates Transform
     * tf appropriately.
     * @param last_odom_pose Transform message containing a recent odometry pose
     * @param scan LaserScan with which to resample the particle filter
     */
    public void updatePF(TransformStamped last_odom_pose, LaserScan scan) {
        // Call user-implemented particle filter update method
        particlecloud = updateParticleCloud(scan, map, particlecloud);
        estimatedpose = updatePoseStamped(particlecloud);

        Time currentTime = node.getCurrentTime();
        
        // Given new estimated pose, now work out the new transform
        tf = recalculateTransform(last_odom_pose, estimatedpose, tf, currentTime);

        // Insert correct timestamp in particlecloud and estimatedpose,
        // so extending subclasses don't need to worry about this, but can
        // just concentrate on updating actual particle and pose locations
        particlecloud.getHeader().setStamp(currentTime);
        particlecloud.getHeader().setFrameId("/map");
        estimatedpose.getHeader().setStamp(currentTime);
        estimatedpose.getHeader().setFrameId("/map");
    }


    /**
     * This should take in a laser scan, map, and current particle cloud.
     * After updating the particle cloud according to the particle filter and
     * the comparison between the observations and the model, it should then
     * return the particle cloud as a PoseArray.
     * @param scan LaserScan message
     * @param map OccupancyGrid describing the world in which the robot is located
     * @param particlecloud PoseArray containing ArrayList of Poses describing current particle cloud
     * @return Updated PoseArray of particles
     */
    public abstract PoseArray updateParticleCloud(LaserScan scan,
            OccupancyGrid map, PoseArray particlecloud);


    /**
     * Given a particle cloud, this should calculate and return an updated robot pose estimate.
     * @param particlecloud PoseArray containing an ArrayList of Poses describing current particle cloud
     * @return PoseWithCovarianceStamped describing robot's estimated position and orientation.
     */
    public abstract Pose updatePose(PoseArray particlecloud);


    public PoseWithCovarianceStamped updatePoseStamped(PoseArray particlecloud) {
        PoseWithCovarianceStamped p = messageFactory.newFromType(PoseWithCovarianceStamped._TYPE);
        p.getPose().setPose(updatePose(particlecloud)); // Get Pose from the implementing class
        p.getPose().setCovariance(getEmptyCov()); // Currently not making use of covariance matrix
        return p;
    }


    /**
     * Creates updated transform from /odom to /map given recent odometry and laser data
     * @param last_odom_pose Last known odometry reading
     * @param estimatedpose Updated pose estimate for the robot
     * @param tf Current transform, which should be updated with new pose information
     * @param currentTime Time stamp for this update (should be the same as the timestamp on the pose estimate)
     */
    public tfMessage recalculateTransform(TransformStamped last_odom_pose, PoseWithCovarianceStamped estimatedpose, tfMessage tf, Time currentTime) {
        
        Transform transform = messageFactory.newFromType(Transform._TYPE);

        // TF should be *difference* between pose (actual, i.e. from laser) and odom position        
        transform.getTranslation().setX(estimatedpose.getPose().getPose().getPosition().getX() - last_odom_pose.getTransform().getTranslation().getX());
        transform.getTranslation().setY(estimatedpose.getPose().getPose().getPosition().getY() - last_odom_pose.getTransform().getTranslation().getY());
        transform.getTranslation().setZ(estimatedpose.getPose().getPose().getPosition().getZ() - last_odom_pose.getTransform().getTranslation().getZ());
        
        transform.getRotation().setW(estimatedpose.getPose().getPose().getOrientation().getW() - last_odom_pose.getTransform().getRotation().getW());
        transform.getRotation().setX(estimatedpose.getPose().getPose().getOrientation().getX() - last_odom_pose.getTransform().getRotation().getX());
        transform.getRotation().setY(estimatedpose.getPose().getPose().getOrientation().getY() - last_odom_pose.getTransform().getRotation().getY());
        transform.getRotation().setZ(estimatedpose.getPose().getPose().getOrientation().getZ() - last_odom_pose.getTransform().getRotation().getZ());
        
        // Insert new Transform into a TransformStamped object and add to the tf tree
        TransformStamped new_tfstamped = messageFactory.newFromType(TransformStamped._TYPE);
        new_tfstamped.setChildFrameId("/odom");
        new_tfstamped.getHeader().setFrameId("/map");
        new_tfstamped.getHeader().setStamp(currentTime);
        //new_tfstamped.getHeader().setSeq(); // TODO Should really maintain and set a seq value...

        // Add the transform to the list of all transforms
        new_tfstamped.setTransform(transform);
        tf.getTransforms().add(new_tfstamped);

        return tf;
    }

    /**
     * Adds the estimated motion from odometry readings to each of the particles in particlecloud
     * @param odomTf Recent Odometry transform
     */
    public void updateOdom(TransformStamped odomTf) {
        double x = odomTf.getTransform().getTranslation().getX();
        double y = odomTf.getTransform().getTranslation().getY();
        double new_heading = getHeading(odomTf.getTransform().getRotation());
        
        // On our first run, the incoming translations may not be equal to zero, so set them appropriately
        if (!odom_initialised) {
            prev_odom_x = x;
            prev_odom_y = y;
            prev_odom_heading = new_heading;
            
            odom_initialised = true;
        }

        // Find difference between current and previous translations
        double dif_x = x - prev_odom_x;
        double dif_y = y - prev_odom_y;
        double dif_heading = new_heading - prev_odom_heading;
        
        // Update previous pure odometry location (i.e. excluding noise) with the new translation
        prev_odom_x = x;
        prev_odom_y = y;
        prev_odom_heading = new_heading;
        
        // Find robot's linear forward/backward motion, given the dif_x and dif_y changes and its orientation
        double distance_travelled = Math.sqrt(dif_x*dif_x + dif_y*dif_y);
        double direction_travelled = Math.atan2(dif_y, dif_x);
        double temp = Math.abs(new_heading - direction_travelled);

        if (temp < -PIOVERTWO || temp > PIOVERTWO) {
            // We are going backwards
            distance_travelled = distance_travelled * -1;
        }
        
        // Random number generator
        Random r = new Random();
        
        // Update each particle with change in position (plus noise)
        for (Pose p : particlecloud.getPoses()) {
            
            double theta = getHeading(p.getOrientation());
            double travel_x = distance_travelled * Math.cos(theta);
            double travel_y = distance_travelled * Math.sin(theta);
            
            double rnd = r.nextGaussian();
            p.getPosition().setX(p.getPosition().getX() + travel_x + (rnd * travel_x * POSITION_NOISE));
            
            rnd = r.nextGaussian();
            p.getPosition().setY(p.getPosition().getY() + travel_y + (rnd * travel_y * POSITION_NOISE));
            
            rnd = r.nextGaussian();
            p.setOrientation(rotateQuaternion(p.getOrientation(), dif_heading + rnd * dif_heading * ROTATION_NOISE));
        }
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
        // Create a temporary Quaternion to represent the change in heading
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
    private static Quaternion multiply_quaternions( Quaternion qa, Quaternion qb ) {
        Quaternion combined = createQuaternion();
        
        combined.setW(qa.getW()*qb.getW() - qa.getX()*qb.getX() - qa.getY()*qb.getY() - qa.getZ()*qb.getZ());
        combined.setX(qa.getX()*qb.getW() + qa.getW()*qb.getX() + qa.getY()*qb.getZ() - qa.getZ()*qb.getY());
        combined.setY(qa.getW()*qb.getY() - qa.getX()*qb.getZ() + qa.getY()*qb.getW() + qa.getZ()*qb.getX());
        combined.setZ(qa.getW()*qb.getZ() + qa.getX()*qb.getY() - qa.getY()*qb.getX() + qa.getZ()*qb.getW());
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
        double yaw = Math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
        return yaw;
    }
    

    public void print_all_orientations(Quaternion q) {
        // Print heading, pitch, bank for a Quaternion. For information only.        
        double w = q.getW();
        double x = q.getX();
        double y = q.getY();
        double z = q.getZ();
        
        double pitch = Math.atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
        double roll = Math.asin(-2*(x*z - w*y)); // Originally, yaw and roll were the other way around
        double yaw = Math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);

        System.out.println("yaw: " + yaw + ", pitch: " + pitch + ", roll: " + roll);

    }

    // Setters
    public void setInitialPose(PoseWithCovarianceStamped pose) {
        this.estimatedpose.setPose(pose.getPose());        
        // Estimated pose has been set, so we should now reinitialise the particle cloud around it
        System.out.println("Got initial pose. Calling initialisePF().");
        particlecloud = initialisePF(estimatedpose);
    }
    
    public void setMap(OccupancyGrid map) {
        this.map = map;
        // Map has changed, so we should reinitialise the particle cloud
        System.out.println("\nGot map. (Re)initialising.");
        particlecloud = initialisePF(estimatedpose);
    }

    public void setTransform(tfMessage tf) {
        this.tf = tf;
    }


    // Getters
    public PoseArray getParticleCloud() {
        return particlecloud;
    }

    public PoseWithCovarianceStamped getPose() {
        return estimatedpose;
    }

    public tfMessage getTransform() {
        return tf;
    }

    public OccupancyGrid getMap() {
        return map;
    }

}
