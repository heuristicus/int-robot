package ex3;

import geometry_msgs.Point;
import java.util.ArrayList;
import org.ros.message.MessageFactory;

public class Vertex {

    ArrayList<Vertex> connectedVertices;
    Point location;

    public Vertex (float x, float y, MessageFactory factory){
        location = factory.newFromType(Point._TYPE);
        location.setX(x);
        location.setY(y);
        connectedVertices = new ArrayList<Vertex>();
    }

    public Vertex(Point location){
        this.location = location;
        connectedVertices = new ArrayList<Vertex>();
    }

    public void setX(float x) {
        location.setX(x);
    }

    public void setY(float y){
        location.setY(y);
    }

    public Point getLocation() {
        return location;
    }

    public ArrayList<Vertex> getConnectedVertices() {
        return connectedVertices;
    }

    public void addConnectedVertex(Vertex v){
        connectedVertices.add(v);
    }

    @Override
    public boolean equals(Object obj) {
        try {
            if (obj == null) {
                return false;
            } else if (this == obj) {
                return true;
            }
            Vertex v = (Vertex) obj;
            return v.getLocation().getX() == this.location.getX() && v.getLocation().getY() == this.location.getY();
        } catch (ClassCastException ex) {
            return false;
        }
    }

    @Override
    public String toString() {
        return "(" + this.location.getX() + ", " + this.location.getY() + ")";
    }



}
