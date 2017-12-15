package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

/**
 * A class that represents a Vertex in a Graph.
 * @author muhirwao
 *
 */
public class GraphNode {
	private GeographicPoint location;
	private List<GraphNode> neighbors;
	
	//Constructor
	public GraphNode(GeographicPoint l){
		this.location = l;
		this.neighbors = new ArrayList<GraphNode>();
	}
	
	//Setter
	/**
	 * Sets the location or GeographicPoint of a node in the map
	 */
	public void setLocation(GeographicPoint l){
		this.location = l;
	}
	
	//Getter
	/**
	 * @return Returns the GeographicPoint of the GraphNode! 
	 */
	public GeographicPoint getLocation(){
		return this.location;
	}
	
	//Add a neighbor
	public void addANeighbor(GraphNode toAdd){
		this.neighbors.add(toAdd);
	}
	
	//Get neighbors
	public List<GraphNode> getNeighbors(){
		return this.neighbors;
	}

}
