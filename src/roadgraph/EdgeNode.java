package roadgraph;

/**
 * A class that represents an edge between two GraphNodes.
 * @author muhirwao
 */
public class EdgeNode {
	private double length;
	private String roadName;
	private String roadType;
	GraphNode from;
	GraphNode to;
	
	//Constructor
	public EdgeNode(GraphNode from, GraphNode to, String roadName,
			String roadType, double length){
		this.length = length;
		this.roadName = roadName;
		this.roadType = roadType;
		this.from = from;
		this.to = to;
	}
	
	//Getters
	public GraphNode getFrom(){
		return this.from;
	}
	
	public GraphNode getTo(){
		return this.to;
	}

}
