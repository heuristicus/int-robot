package util;

import java.util.Arrays;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import sensor_msgs.LaserScan;


public class LaserUtil extends AbstractNodeMain {

    public static final float DEFAULT_ZERO_REPLACEMENT = 7.0f;

    @Override
    public void onStart(ConnectedNode node) {
    Subscriber<LaserScan> laser;
    laser = node.newSubscriber("base_scan", LaserScan._TYPE);

        laser.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan scan) {
                float[] ranges = scan.getRanges();
                for (int i = 0; i < ranges.length; i++) {
                    System.out.print(i + ": " + ranges[i] + ", ");
                    if (i % 20 == 0) {
                        System.out.println("");
                    }
                }

                System.out.println("\n\nSEPARATOR----------------------------\n\n");

                float[][] sectors = getSectors(5, scan);
                for (int sector = 0; sector < sectors.length; sector++) {
                    System.out.println("Sector "+sector+" ---------");
                    for (int i = 0; i < sectors[sector].length; i++) {
                        System.out.println(i + ": " + sectors[sector][i]);
                    }
                    System.out.println();
                }
                System.out.println("\n\n");

                float[] avgs = averageOfEachSector(sectors);
                for (int i=0; i < avgs.length; i++) {
                    System.out.println("Avg "+i+": "+avgs[i]);
                }

                System.out.println("\n\n");
                System.out.println("AngleIncrement: " + scan.getAngleIncrement());
                System.out.println("AngleMax: " + scan.getAngleMax());
                System.out.println("AngleMin: " + scan.getAngleMin());
                System.exit(0);
            }
        });
    }

    /**
     * Best not to use this method. Use getSectors() instead. See javadoc there.
     */
    private static float[][] _getSectors(int numOfSectors, int degreesPerSector, LaserScan laser) {
        if (numOfSectors % 2 == 0) {
            throw new IllegalArgumentException("Only odd numbers accepted.");
        }

        float[] ranges = laser.getRanges();

        float[][] sectors = new float[numOfSectors][degreesPerSector];

        int degreesRequired = numOfSectors * degreesPerSector;
        if (ranges.length < degreesRequired + 1) { // +1 is because of the way we get the central readings.
            throw new IllegalStateException("Oh noez, too many sectors requested");
        }

        int middleAngle = ranges.length / 2;
        int startAngle = middleAngle - (degreesRequired / 2);
        int readingIndex;
        float readingVal;
        for (int sector = 0; sector < numOfSectors; sector++) {
            for (int reading = 0; reading < sectors[sector].length; reading++) {
                readingIndex = startAngle + (degreesPerSector * sector) + reading;
                if (ranges[readingIndex] == 0.0f){
                    readingVal = DEFAULT_ZERO_REPLACEMENT;
                } else {
                    readingVal = ranges[readingIndex];
                }
                //sectors[sector][reading] = ranges[readingIndex] == 0.0 ? DEFAULT_ZERO_REPLACEMENT : ranges[readingIndex];
                sectors[sector][reading] = readingVal;
            }

            //System.arraycopy(ranges, startAngle + (degreesPerSector * sector), sectors[sector], 0, sectors[sector].length);
        }
        return sectors;
    }

    /**
     * There will be 64 sectors over the 180 degree viewing angle of the
     * laser. These will each contain 8 readings covering 2.8125 degrees each.
     * Call this method with how many sectors you would like from the central
     * section of the laser's viewing angle. Note that the current implementation
     * does not actually allow 64 sectors, since the sectors are based around
     * the centre of the viewing angle. E.g. if you ask for 1 sector you get
     * the central few degrees, not left of centre or right of centre. So some
     * degrees get eaten up. Max num of sectors requestable therefore is 63.
     */
    public static float[][] getSectors(int numOfSectors, LaserScan laser) {
        return _getSectors(numOfSectors, 8, laser);
    }


    /* gets the median of the specified sector. Use the get */
    private static float _getSectorMedian(float[] sector) {
        Arrays.sort(sector);
        int len = sector.length;
        int mid = len / 2;
        if (len % 2 == 0) {
        	// If even, get middle two values and return their average.
            return (sector[mid] + sector[mid - 1]) / 2;
        } else {
            // just return the median value
            return sector[mid];
        }

    }

    public static float[] medianOfEachSector(float[][] sectors){
        float[] medians = new float[sectors.length];
        for (int i = 0; i < sectors.length; i++) {
            medians[i] = _getSectorMedian(sectors[i]);
        }
        return medians;
    }


    public static float[] averageOfEachSector(float[][] readings)
            throws IllegalStateException {
        float[] averages = new float[readings.length];

        final float readingsPerSector = readings[0].length;
        float totalForSector;

        for (int sector = 0; sector < readings.length; sector++) {
            totalForSector = 0;
            for (int i = 0; i < readingsPerSector; i++) {
                totalForSector += readings[sector][i];
            }
            averages[sector] = totalForSector / readingsPerSector;
        }
        return averages;
    }

    public static int minReadingPos(float[] readings, float ignoreThreshold) {
        if (readings.length < 1) throw new IllegalArgumentException("Gimme args, yo");
        float min = Float.MAX_VALUE;
        int minPos = 0; // Position of the minimum found so far
        float current;

        for (int i = 0; i < readings.length; i++) {
            current = readings[i];

            if (current < ignoreThreshold){
                System.out.println("VALUE BELOW THRESHOLD --- sector " + i + " with value " + current);
                continue;
            } else if (current < min) {
                min = current;
                minPos = i;
            }
        }
        if (min == Float.MAX_VALUE) {
            System.out.println("SOMETHING WENT HORRIBLY WRONG");
        }

        return minPos;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LaserUtil");
    }

}
