package util.laserplot;

import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JPanel;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import sensor_msgs.LaserScan;

public class VisualLaserPlot extends AbstractNodeMain {

    public static final Color BACKGROUND_COLOUR = Color.black;
	
    // The range of the laser (so we know where to plot up to
    public static final float RANGE_MIN = 0.02f;
    public static final float RANGE_MAX = 5.60f;
	
    // Regular points plotted on the chart
    public static final int PLOT_POINT_WIDTH = 2;
    public static final int PLOT_POINT_HEIGHT = 2;
    public static final Color PLOT_POINT_COLOR = Color.white;

    // If we get a zero we plot it differently for emphasis
    public static final int ZERO_POINT_WIDTH = 5;
    public static final int ZERO_POINT_HEIGHT = 5;
    public static final Color ZERO_POINT_COLOR = Color.red;

    JPanel panel;
    float[] latestReadings = new float[512];
	
    // Size of the window/panel
    public int sizeX = latestReadings.length;
    public int sizeY = (int) (RANGE_MAX * 100.0f);

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("VisualLaserPlot");
    }

    @Override
    public void onStart(ConnectedNode node) {
    	panel = new LaserPlotPanel();
        panel.setSize(sizeX, sizeY);
    	panel.setBackground(BACKGROUND_COLOUR);
    	
        Subscriber<LaserScan> laser = node.newSubscriber("base_scan", LaserScan._TYPE);

        laser.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan laser) {
            	latestReadings = laser.getRanges();
            	panel.repaint();
            }
        });
        
        LaserPlotWindow window = new LaserPlotWindow(panel, sizeX, sizeY);
        window.setVisible(true);
    }
    
    @SuppressWarnings("serial")
    public class LaserPlotPanel extends JPanel {
        @Override
    	public void paint(Graphics g) {
            g.setColor(BACKGROUND_COLOUR);
            g.fillRect(0, 0, sizeX*2, sizeY*2);
            g.setColor(PLOT_POINT_COLOR);
            float reading;
            int y;

            for (int x = 0; x < latestReadings.length; x++) {
                reading = latestReadings[x];
                y = sizeY-((int)(reading * 100.0f));
                
                if (reading != 0) {
                    g.fillOval(x, y, PLOT_POINT_WIDTH, PLOT_POINT_HEIGHT);
                } else { // We draw zeros differently
                    g.setColor(ZERO_POINT_COLOR);
                    g.fillOval(x, y, ZERO_POINT_WIDTH, ZERO_POINT_HEIGHT);
                    g.setColor(PLOT_POINT_COLOR);
                }
            }
    	}
    }
}
