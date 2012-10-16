package pf;

import org.jboss.netty.buffer.ChannelBuffer;
import geometry_msgs.Pose;
import nav_msgs.OccupancyGrid;
import sensor_msgs.LaserScan;

/**
 *
 * @author rowanms
 * Ported from AMCL's sensors/amcl_laser.cpp code
 */
public class SensorModel {
    
    /*
    Defaults from AMCL config:
    gen.add("laser_z_hit", double_t, 0, "Mixture weight for the z_hit part of the model.", .95, 0, 10)
    gen.add("laser_z_short", double_t, 0, "Mixture weight for the z_short part of the model.", .1, 0, 10)
    gen.add("laser_z_max", double_t, 0, "Mixture weight for the z_max part of the model.", .05, 0, 10)
    gen.add("laser_z_rand", double_t, 0, "Mixture weight for the z_rand part of the model.", .05, 0, 10)
    
    gen.add("laser_sigma_hit", double_t, 0, "Standard deviation for Gaussian model used in z_hit part of the model.", .2, 0, 10)
    gen.add("laser_lambda_short", double_t, 0, "Exponential decay parameter for z_short part of model.", .1, 0, 10)
    gen.add("laser_likelihood_max_dist", double_t, 0, "Maximum distance to do obstacle inflation on map, for use in likelihood_field model.", 2, 0, 20)
    */
    
    private static double z_hit = 0.95; // Default probability if we make a hit
    private static double z_short = 0.1; // Probability of a short reading from unexpected obstacle (e.g. person or object)
    private static double z_max = 0.05; // Probability of failure to detect an obstacle, reported as max range
    private static double z_rand = 0.05; // Random noise on all readings
    
    private static double sigma_hit = 0.2; // Noise on hit
    private static double lambda_short = 0.1; // Noise on short reading
    
    private static final double PIOVERTWO = Math.PI/2; // To speed up calculations
    
    /**
     * Compute the likelihood weighting for each of a set of particles 
     * @param scan LaserScan message showing current observations
     * @param pose Particle's estimated location
     * @param map OccupancyGrid message containing the map
     * @param numReadings Number of scan readings to be compared with predictions
     * @return likelihood weighting for this particle, given the map and laser readings
     */
    public static double getWeight(LaserScan data, OccupancyGrid map, Pose pose, int numReadings) {

        double p = 1.0; // Sample weight (not a probability!)
        
        int step = (data.getRanges().length - 1) / (numReadings - 1); // Step-through size for ranges[]
        // E.g. 100 data items, 5 predicted readings: (int)99/4 = 24, so readings are taken at 0, 24, 48, 72, 96.
        
        for (int i = 0; i < data.getRanges().length; i += step) {
            // For each range...
            double obs_range = data.getRanges()[i];
            // Bearing relative to the robot
            double obs_bearing = data.getAngleMin() + ((data.getAngleMax() - data.getAngleMin()) * ((double)i / (double)data.getRanges().length));
            //System.out.println("Particle heading: " + AbstractLocaliser.getHeading(pose.orientation) + ", i: " + i + ", bearing: " + obs_bearing);
            if (obs_range <= 0.0) {
                obs_range = data.getRangeMax(); // Laser reports max range as zero, so set it to range_max
            }
            
            // Compute the range according to the map
            double map_range = calcMapRange(pose.getPosition().getX(), pose.getPosition().getY(), AbstractLocaliser.getHeading(pose.getOrientation()) + obs_bearing, data.getRangeMax(), map);
            double pz = predict(obs_range, map_range, data.getRangeMax());
            p += pz*pz*pz; // Cube probability to drastically reduce low-probability particles 
            
        }
        return p;
    }
    
    /**
     * Implementation of AMCL's sensor model.
     * @param obs_range Observation range (i.e. from actual laser data)
     * @param map_range Predicted range (i.e. from map)
     * @param range_max Laser's maximum range
     * @return Probability (0-1) that we would observe the obs_range data given the estimated map location
     */
    public static double predict(double obs_range, double map_range, double range_max) {
        double pz = 0.0;

        // Part 1: good, but noisy, hit
        double z = obs_range - map_range;
        pz += z_hit * Math.exp(-(z * z) / (2 * sigma_hit * sigma_hit));
        
        // Part 2: short reading from unexpected obstacle (e.g., a person)
        if(z < 0) {
            pz += z_short * lambda_short * Math.exp(-lambda_short*obs_range);
        }

        // Part 3: Failure to detect obstacle, reported as max-range
        if(obs_range == range_max) {
            pz += z_max * 1.0;
        }

        // Part 4: Random measurements
        if(obs_range < range_max) {
            pz += z_rand * 1.0/range_max;
        }
        
        assert(pz <= 1.0);
        assert(pz >= 0.0);
        
        return pz;
    }

    /**
     * Given a location on the map and a direction, predict the visible laser range.
     * @param ox X location of observation
     * @param oy Y location of observation
     * @param bearing Bearing (from North, in degrees) of the reading
     * @param range_max Laser's maximum range
     * @param map Occupancy grid map of the world
     * @return Range (in m) expected to be observed by the laser
     */
    public static double calcMapRange(double ox, double oy, double bearing, double range_max, OccupancyGrid map) {
        ChannelBuffer data = map.getData();
        
         // As the ROS angle system has 0 deg = east and increases anti-clockwise
        // but the Quaternion trigonometry and particle raytracing assume 0 deg = north / clockwise,
        // we must first convert to 0 deg = north / clockwise by subtracting PI/2
        bearing -= PIOVERTWO;
        bearing *= -1;

        // Map data
        long map_width = map.getInfo().getWidth();
        long map_height = map.getInfo().getHeight();
        float map_resolution = map.getInfo().getResolution(); // in m per pixel

        // Find gradient of the line of sight in x,y plane, assuming 0 deg = north
        double grad_x = Math.sin(bearing);
        double grad_y = Math.cos(bearing);

        // Particle position
        double x_orig = ox / map_resolution;
        double y_orig = oy / map_resolution;
        // Max range position relative to the current position
        double x_max_offset = range_max * grad_x / map_resolution;
        double y_max_offset = range_max * grad_y / map_resolution;

        //                System.out.println("Orientation: " + orientation + ", reading angle: " + READING_ANGLES[reading] +
        //                        "\nLOS angle: " + LOSangle + ", grad_x: " + grad_x + ", grad_y: " + grad_y);
        double x = x_orig;
        double y = y_orig;
        boolean occupied = false; // Have we found an occupied cell yet?

        // Stop travelling away from the robot when we reach max range of
        // laser or an occupied cell
        while (Math.abs(x-x_orig) < Math.abs(x_max_offset) &&
                Math.abs(y-y_orig) < Math.abs(y_max_offset) &&
                        !occupied) {
            //                    System.out.println("distance: " + distance + ", max: " + max_range / map_resolution);
            //   find x,y position of next cell along the line of sight
            x += grad_x;
            y += grad_y;
//            System.out.println("x: " + x + ", y: " + y);
//            System.out.println("x orig: " + x_orig + ", xmax: " + x_max_offset);
//            System.out.println("y orig: " + y_orig + ", ymax: " + y_max_offset);
            
            //   if cell occupied (i.e. probability of occupancy > 50)...

            //                    System.out.println("direction: " + LOSangle);
            //                    System.out.println("x_loc: " + x_loc + ", y_loc: " + y_loc);

            //int index = getMapIndex(map_origin_x + x_loc + nextCellX,
            //        map_origin_y + y_loc + nextCellY, (int)map_width);
            int index = getMapIndex((int)Math.round(x), (int)Math.round(y), (int) map_width, (int) map_height);

            if (index > 0 && index < data.capacity()) {
                // If we are on the map to begin with...
                Byte cellData = data.getByte(index);

                if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                    // If we're on the map, but the map has no data, or there is an obstacle...
                    occupied = true;
                } else {
                    occupied = false;
                }
            } else {
                occupied = true;
            }
        }
        // System.out.println("x: " + x + ", x orig: " + x_orig + ", x max: " + x_max_offset);

        // System.out.println("orientation: " + orientation + " LOS: " + LOSangle);
        // System.out.println("reading : " + reading + ", angle: " + READING_ANGLES[reading] + ", distance (px): " + distance + ", distance (m) " + distance * map_resolution);
        double distance = Math.sqrt((x-x_orig)*(x-x_orig) + (y-y_orig)*(y-y_orig)) * map_resolution;
        //            System.out.println("-------------- NEXT PARTICLE -----------------");
        //            System.out.println("map resolution: " + map_resolution);
        return distance;
    }
    

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
