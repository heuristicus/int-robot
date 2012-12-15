/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4.viewer;

import geometry_msgs.Point;
import java.awt.BorderLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 *
 * @author robot
 */
public class MapViewerNode extends AbstractNodeMain {

    private ConnectedNode node;
    private MessageFactory messageFactory;
    private Subscriber<OccupancyGrid> originalMapSub;
    private Subscriber<OccupancyGrid> heatMapSub;
    private Subscriber<Odometry> positionSub;
    private OccupancyGrid originalMap;
    private JFrame frame;
    private MapViewerJPanel panel;
    private MessageFactory factory;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("MapViewer");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        frame = new JFrame("Map viewer");
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        factory = connectedNode.getTopicMessageFactory();

        originalMapSub = connectedNode.newSubscriber("map", OccupancyGrid._TYPE);
        originalMapSub.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid t) {
                if (originalMap == null) {
                    originalMap = t;
                    panel = new MapViewerJPanel(t.getInfo().getWidth(), t.getInfo().getHeight(), t.getInfo().getResolution());
                    System.out.println("Map Width:" + t.getInfo().getWidth() + " map Height:" + t.getInfo().getHeight());
                    panel.setMap(t.getData().array());

                    frame.setLayout(new BorderLayout());
                    frame.add(panel, BorderLayout.CENTER);
                    JButton heat = new JButton("Heatmap");
                    JButton path = new JButton("Path");
                    panel.listenTo(path);
                    panel.listenTo(heat);
                    JPanel bpan = new JPanel();
                    bpan.add(heat);
                    bpan.add(path);
                    frame.add(bpan, BorderLayout.SOUTH);
                    frame.pack();
                    frame.setSize(t.getInfo().getWidth() + 5, t.getInfo().getHeight() + 60);
                    frame.setVisible(true);
                    frame.setLocationRelativeTo(null);
                    panel.repaint();
                }
            }
        });

        heatMapSub = connectedNode.newSubscriber("heat_map", OccupancyGrid._TYPE);
        heatMapSub.addMessageListener(new MessageListener<OccupancyGrid>() {

            @Override
            public void onNewMessage(OccupancyGrid t) {
                if(panel != null){
                    panel.setHeatMap(t.getData().array());
                    panel.repaint();
                }
            }
        });

        positionSub = connectedNode.newSubscriber("base_pose_ground_truth", Odometry._TYPE);
        positionSub.addMessageListener(new MessageListener<Odometry>() {

            @Override
            public void onNewMessage(Odometry t) {
                Point p = factory.newFromType(Point._TYPE);
                p.setY(t.getPose().getPose().getPosition().getX() - 0.6);
                p.setX(-t.getPose().getPose().getPosition().getY() - 0.6);
                panel.addPathPoint(p);
            }
        });
    }
}
