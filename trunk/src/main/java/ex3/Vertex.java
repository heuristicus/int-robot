package ex3;

import java.util.ArrayList;

public class Vertex {

    ArrayList<Edge> edges;
    double x;
    double y;

    public Vertex (double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

}
