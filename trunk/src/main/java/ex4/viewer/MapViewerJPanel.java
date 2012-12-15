/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4.viewer;

import geometry_msgs.Point;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import javax.swing.JButton;
import javax.swing.JPanel;

/**
 *
 * @author robot
 */
public class MapViewerJPanel extends JPanel implements ActionListener {

    private int mapHeight, mapWidth;
    private float mapRes;
    private byte[] mapData;
    private byte[] heatMapData;
    private ArrayList<Point> pathData;
    boolean path = true;
    boolean heat = true;

    public MapViewerJPanel(int mapHeight, int mapWidth) {
        this.mapHeight = mapHeight;
        this.mapWidth = mapWidth;
        this.setSize(mapWidth, mapHeight);
        this.pathData = new ArrayList<Point>();
    }

    public MapViewerJPanel(int mapHeight, int mapWidth, float mapRes){
        this.mapRes = mapRes;
        this.mapHeight = mapHeight;
        this.mapWidth = mapWidth;
        this.setSize(mapWidth, mapHeight);
        this.pathData = new ArrayList<Point>();
    }

    public void listenTo(JButton b){
        b.addActionListener(this);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if ("Heatmap".equals(e.getActionCommand())){
            heat = !heat;
        } else if ("Path".equals(e.getActionCommand())){
            path = !path;
        }
        repaint();
    }

    public void setMap(byte[] mapData) {
        this.mapData = mapData;
    }

    public void setHeatMap(byte[] heatMapData) {
        this.heatMapData = heatMapData;
    }

    public void addPathPoint(Point p){
        pathData.add(p);
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        //paint original map
        if (mapData != null) {
            for (int x = 0; x < mapWidth; x++) {
                for (int y = 0; y < mapHeight; y++) {
                    int index = getMapIndex(x, y);
                    if (index > 0 && index < mapData.length) {
                        int cellData = mapData[index];
                        if (cellData < 0) {
                            //unknown
                            g.setColor(Color.GRAY);
                        } else if (cellData <= 65) {
                            //unoccupied
                            g.setColor(Color.LIGHT_GRAY);
                        } else {
                            //occupied
                            g.setColor(Color.BLACK);
                        }
                        drawPixel(g, x, y, 100);
                    }
                }
            }
        }

        if (heat) {
            //paint heat map
            if (heatMapData != null) {
                byte[] heatDataCopy = (byte[]) heatMapData.clone();
                //get max heat value
                double maxHeatValue = 0;
                for (double heatValue : heatDataCopy) {
                    if (heatValue > maxHeatValue) {
                        maxHeatValue = heatValue;
                    }
                }

                for (int x = 0; x < mapWidth; x++) {
                    for (int y = 0; y < mapHeight; y++) {
                        int index = getMapIndex(x, y);
                        if (index > 0 && index < heatDataCopy.length) {
                            int heatValue = heatDataCopy[index];
                            if (heatValue > 0) {
//                            System.out.println("Heat value: " + heatValue);
                                g.setColor(getHeatColor(heatValue, maxHeatValue));
                                drawPixel(g, x, y, 100);
                            }
                        }
                    }
                }
            }
        }

        if (path) {
            if (pathData != null) {
                for (Point p : pathData) {
                    g.setColor(Color.black);
                    drawPixel(g, (int) (p.getX() / mapRes), (int) (p.getY() / mapRes), 0);
                }
            }
        }
    }

    private int getMapIndex(int x, int y) {
        if (x < 0 || y < 0 || x > mapWidth || y > mapHeight) {
            return -1;
        } else {
            return (y * mapWidth) + x;
        }
    }

    private void drawPixel(Graphics g, int x, int y, int offset) {
        //mapHeight - y: flip map
        if (x <= offset) {
            g.drawLine(mapWidth - offset + x, mapHeight - y, mapWidth - offset + x, mapHeight - y);
        } else {
            g.drawLine(x - offset, mapHeight - y, x - offset, mapHeight - y);
        }
    }

    private Color getHeatColor(double value, double maxValue) {
        //colour
        float minHue = 0; //blue;
        float maxHue = 0.6875f; //red
        float hue = maxHue - ((float) (value / maxValue) * (maxHue - minHue));
        return Color.getHSBColor(hue, 1, 1);
        //monotone
        //    int colour = (int)((value / maxValue) * 255.0);
        //    return new Color(colour, 0, 0);
    }



}
