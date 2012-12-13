/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4.viewer;

import ex4.Printer;
import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JPanel;

/**
 *
 * @author robot
 */
public class MapViewerJPanel extends JPanel {

    private int mapHeight, mapWidth;
    private byte[] mapData;
    private byte[] heatMapData;

    public MapViewerJPanel(int mapHeight, int mapWidth) {
        this.mapHeight = mapHeight;
        this.mapWidth = mapWidth;
        this.setSize(mapWidth, mapHeight);
    }

    public void setMap(byte[] mapData) {
        this.mapData = mapData;
    }

    public void setHeatMap(byte[] heatMapData) {
        this.heatMapData = heatMapData;
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
                        int offset = 100;
                        if (x <= offset) {
                            g.drawLine(mapWidth - offset + x, y, mapWidth - offset + x, y);
                        } else {
                            g.drawLine(x - offset, y, x - offset, y);
                        }
                    }
                }
            }
        }

        //paint heat map
        if (heatMapData != null) {
            byte[] heatDataCopy = (byte[])heatMapData.clone();
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
                            g.setColor(getHeatColor(heatValue, maxHeatValue));

                            int offset = 100;
                            if (x <= offset) {
                                g.drawLine(mapWidth - offset + x, y, mapWidth - offset + x, y);
                            } else {
                                g.drawLine(x - offset, y, x - offset, y);
                            }
                        }
                    }
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

    private Color getHeatColor(double value, double maxValue) {
        //colour
        float minHue = 0; //blue;
        float maxHue = 0.6875f; //red
        float hue = maxHue - ((float) (value / maxValue) * (maxHue- minHue));
        return Color.getHSBColor(hue, 1, 1);
        //monotone
    //    int colour = (int)((value / maxValue) * 255.0);
    //    return new Color(colour, 0, 0);
    }
}
