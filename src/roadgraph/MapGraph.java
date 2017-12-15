/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private Map<GraphNode, ArrayList<GraphNode>> map; //Graph is represented as an adjancyList
	private Set<EdgeNode> edges;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		this.map = new HashMap<GraphNode, ArrayList<GraphNode>>();
		this.edges = new HashSet<EdgeNode>();
		this.numEdges = 0;
		this.numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return this.numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> ret = new HashSet<GeographicPoint>();
		for(GraphNode g : map.keySet()){
			ret.add(g.getLocation());
		}
		return ret;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return this.numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(this.getVertices().contains(location)){
			return false;
		}else{
			this.numVertices += 1;
			GraphNode g = new GraphNode(location);
			this.map.put(g, new ArrayList<GraphNode>());
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 3
		if(!initializeAddEdge(from, to, roadName, roadType, length)){
			throw new IllegalArgumentException();
		}
		//Get GraphNodes
		GraphNode start = getGraphNodeFromLocation(from);
		GraphNode end = getGraphNodeFromLocation(to);
		
		//Update Neighbors
		start.addANeighbor(end);
		map.get(start).add(end);
		
		//Add Edge
		EdgeNode e = new EdgeNode(start, end, roadName, roadType, length);
		edges.add(e);
		this.numEdges += 1;
	}
	
	/**
	 * This method initializes the addEdge Method!
	 * @param from GeographicPoint where the edge starts
	 * @param to GeographicPoint where the edge ends
	 * @param roadName A String containing the name of the road
	 * @param roadType A String containing the type of the road
	 * @param length A Double containing the length of the road
	 * @return A boolean to show if the parameters passed to the method have been accepted or not
	 */
	private boolean initializeAddEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length){
		if(!getVertices().contains(from) || !getVertices().contains(to)){
			return false;
		}
		//if any of the arguments is null
		if(from == null || to == null || roadName == null || roadType == null){
			return false;
		}
		if(length < 0){
			return false;
		}
		return true;
	}
	
	/**
	 * This method return the GraphNode that's associated with a GeographicPoint
	 * @param l The Location whose GraphNode needs to be found.
	 * @return Null or GraphNode that's associated with a location!
	 */
	private GraphNode getGraphNodeFromLocation(GeographicPoint l){
		GraphNode ret = null;
		for(GraphNode g : map.keySet()){
			if(g.getLocation().equals(l)){
				ret = g;
				break;
			}
		}
		return ret;
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		//Initialize BFS
		GraphNode s = getGraphNodeFromLocation(start);
		GraphNode g = getGraphNodeFromLocation(goal);
		
		Map<GraphNode, GraphNode> parentMap = new HashMap<GraphNode, GraphNode>();
		
		//searh
		boolean found = search(s, g, parentMap, nodeSearched);	
		
		//Build Path
		if(!found){
			return new ArrayList<GeographicPoint>();
		}
		//Need List of GeographicPoints to return
		return buildPath(s, g, parentMap);
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		
	}
	
	/**
	 * Private method that performs BFS. While it's searching, it's also capturing the map of what nodes are
	 * connected to each other!
	 * @param s GraphNode where the search starts from
	 * @param g GraphNode that's the goal of our search. Once found, it should return true.
	 * @param parentMap
	 * @return
	 */
	private boolean search(GraphNode s, GraphNode g, Map<GraphNode, GraphNode> parentMap,
							Consumer<GeographicPoint> nodeSearched){
		Queue<GraphNode> q = new LinkedList<GraphNode>();
		Set<GraphNode> visited = new HashSet<GraphNode>();
		boolean found = false;
		
		//Start searching
		q.add(s);
		visited.add(s);
		while(!q.isEmpty()){
			GraphNode curr = q.remove();
			if(curr == g){
				found = true;
				break;
			}
			for(GraphNode n : curr.getNeighbors()){
				if(!visited.contains(n)){
					visited.add(n);
					parentMap.put(n, curr);
					q.add(n);
				}
			}
			nodeSearched.accept(curr.getLocation());
		}
		
		return found;
	}
	
	/**
	 * Private method used by BFS to build path
	 * @param start GraphNode where the path starts
	 * @param goal GraphNode where the path should end
	 * @param parentMap A map that maps GraphNodes that are connected to each other
	 * @return The a list of GeographicPoints from start to goal
	 */
	private List<GeographicPoint> buildPath(GraphNode start, GraphNode goal, Map<GraphNode, GraphNode> parentMap){
		LinkedList<GeographicPoint> ret = new LinkedList<GeographicPoint>();
		GraphNode curr = goal;
		while(curr != start){
			ret.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		ret.addFirst(start.getLocation());
		return ret;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		GeographicPoint start = new GeographicPoint(8.0, -1.0);
		GeographicPoint end = new GeographicPoint(4.0, 2.0);
//		System.out.println(bfs());
		int k = firstMap.bfs(start, end).size();
		System.out.println(k);
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
