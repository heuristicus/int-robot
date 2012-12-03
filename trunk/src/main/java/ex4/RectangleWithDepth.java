/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import java.awt.Rectangle;

/**
 *
 * @author robot
 */
public class RectangleWithDepth extends Rectangle.Double {

    double depth;

    public RectangleWithDepth(double x, double y, double w, double h, double depth) {
        super(x, y, w, h);
        this.depth = depth;
    }

    public double getDepth() {
        return depth;
    }
}
