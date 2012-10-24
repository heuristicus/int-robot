package ex3;

import geometry_msgs.Point32;
import java.util.ArrayList;
import org.ros.message.MessageFactory;

public class Vertex {

    ArrayList<Edge> edges;
    Point32 location;

    public Vertex (float x, float y, MessageFactory factory){
        location = factory.newFromType(Point32._TYPE);
        location.setX(x);
        location.setY(y);
    }

    public Vertex(Point32 location){
        this.location = location;
    }

    public void setX(float x){
        location.setX(x);
    }

    public void setY(float y){
        location.setY(y);
    }

    public Point32 getLocation() {
        return location;
    }

    public ArrayList<Edge> getEdges() {
        return edges;
    }

}
