package ex3.search;

import ex3.PRMGraph;
import ex3.PRMUtil;
import ex3.Vertex;
import java.util.LinkedList;
import java.util.HashMap;

public class Dijkstra implements SearchAlgorithm {

    @Override
    public LinkedList<Vertex> shortestPath(Vertex v1, Vertex v2, PRMGraph graph, PRMUtil util) {
        HashMap<Vertex, DijkstraTuple> map = new HashMap<Vertex, DijkstraTuple>(); // Tuple graph
        HashMap<Vertex, DijkstraTuple> unchecked = new HashMap<Vertex, DijkstraTuple>();

        // Create tuples for each point in graph. Add everything to unchecked.
        for (Vertex inGraph : graph.getVertices()) {
            map.put(inGraph, new DijkstraTuple(Double.MAX_VALUE, null));
            unchecked.put(inGraph, new DijkstraTuple(Double.MAX_VALUE, null));
	}

        map.get(v1).setDistance(0);
        unchecked.get(v1).setDistance(0);

	while(!unchecked.isEmpty()){

            System.out.println(" unchecked size: " + unchecked.size());

	    Vertex minVertex = shortestDistance(unchecked);

            // Break if remaining vertices inaccessible from this node
	    if (minVertex == null){
//                System.out.println("Cannot access other vertices from this vertex...");
		break;
	    }

            DijkstraTuple vData = unchecked.remove(minVertex);

            System.out.println(minVertex + " goal " + v2);

            //System.out.println("Removed " + minVertex + " with tentative distance " + vData.getDistance());
	    if (minVertex.equals(v2)){ // If we found the destination
                System.out.println("Found the goal vertex!");
		LinkedList<Vertex> path = new LinkedList<Vertex>();
		path.addFirst(minVertex); // Add the end vertex to the list
		
		Vertex current = minVertex;
		DijkstraTuple curData = vData;
		
                // Go back through the list and construct the path
		while (curData.getPrevious() != null){
		    current = curData.getPrevious();
		    curData = map.get(current);
		    
		    path.addFirst(current);
		}

                for (Vertex vertex : path) {
                    System.out.print(vertex + "->");
                }
                System.out.println("");

		return path;
		
	    }
	    
	    
	    
	    for (Vertex neighbour : minVertex.getConnectedVertices()){
		if (!unchecked.containsKey(neighbour)){
//                    System.out.println("Skipping already visited vertex.");
		    continue; // skip the node if we've already visited it.
		}
		
		// Get the distance through the current vertex to the neighbour
		double altDist = vData.getDistance() + util.getEuclideanDistance(minVertex, neighbour);
//                System.out.println("Distance through this vertex to vertex at " + neighbour + " is " + altDist);
		DijkstraTuple mapData = map.get(neighbour);
		DijkstraTuple uncheckedData = unchecked.get(neighbour);
                
		// If the distance through the current node to the neighbour is shorter than the
		// current tentative distance we have for that neighbour
		if (altDist < mapData.getDistance()){
//                    System.out.println(altDist + " is smaller than " + mapData.getDistance() + " updating.");
                    mapData.setDistance(altDist);
		    mapData.setPrevious(minVertex);

                    uncheckedData.setDistance(altDist);
                    uncheckedData.setPrevious(minVertex);
		} else {
//                    System.out.println("No improvement on path through this vertex.");
                }
		
	    }
	}
	
	return null;
	
    }

    /*
     * Finds the node in the map which has the shortest distance.
     */
    private Vertex shortestDistance(HashMap<Vertex, DijkstraTuple> map){
	double minDist = Double.MAX_VALUE;
	double curDist = minDist;
	Vertex minVertex = null;
	
	for (Vertex dt : map.keySet()){
	    curDist = map.get(dt).getDistance();
	    if (curDist < minDist){
		minDist = curDist;
		minVertex = dt;
	    }
	    
	}
	return minVertex;
    }

}
