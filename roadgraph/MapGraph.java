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
import java.util.PriorityQueue;
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
	//TODO: Add your member variables here in WEEK 3
	//private final int defaultNumVertices = 5;
	private Map<GeographicPoint,VerticeNode> vertices;//stores the location and the vertice related to it
	private int NumVertices;//counts no of Vertices
	private int NumEdges;//counts no of Edges
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<GeographicPoint,VerticeNode>();
		NumVertices = 0;
		NumEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return NumVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		return new HashSet<GeographicPoint>(vertices.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return NumEdges;
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
		if(vertices.containsKey(location)||location == null) {
			return false;
		}
		NumVertices++;//increament the count of vertices by 1
		vertices.put(location,new VerticeNode(location));
		return true;
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
		if(!(vertices.containsKey(from)||vertices.containsKey(to))||roadName==null
				||roadType == null|| length< 0.0){
			throw new IllegalArgumentException("Illegal Entry");
		}
		VerticeNode From = vertices.get(from);
		VerticeNode To = vertices.get(to); 
		From.addNeighbor(To, roadName, roadType, length);
		NumEdges++;//increament the no of edges by 1
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
	private boolean BFSSearch(VerticeNode start,VerticeNode goal,
			Map parentMap,Consumer<GeographicPoint> nodeSearched) {
		HashSet<VerticeNode> visited = new HashSet<VerticeNode>();
		Queue<VerticeNode> toExplore = new LinkedList<VerticeNode>();
		toExplore.add(start);
		boolean found = false;
	
		while(!toExplore.isEmpty()) {
			VerticeNode currVertice = toExplore.remove();
			//System.out.println("VerticeNode: "+currVertice.getPoint());
			
			nodeSearched.accept(currVertice.getPoint());
			if(currVertice == goal) {
				found = true;
				break;
			}
			List<EdgeNode> neighbors = currVertice.getNeighbors();
			
			for(EdgeNode neighbor: neighbors) {
				//System.out.println("Neighbor: "+neighbor.getOtherEnd().getPoint());
				VerticeNode nxtVertice = neighbor.getOtherEnd();
				if(!visited.contains(nxtVertice)) {
					visited.add(nxtVertice);
					parentMap.put(nxtVertice, currVertice);
					toExplore.add(nxtVertice);
					
				}
				
			}
			
		}
		//System.out.println("Execution of while loop: "+i);
		//System.out.println(found);
		return found;
	}
	private static List<GeographicPoint> constructPath(VerticeNode start,
			VerticeNode goal,Map<VerticeNode,VerticeNode> parentMap){
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		VerticeNode curr = goal;
		while(curr!=start) {
			GeographicPoint currLocation = curr.getPoint();
			//System.out.println(currLocation.toString());
			path.addFirst(currLocation);
			curr = parentMap.get(curr);
		}
		path.addFirst(start.getPoint());
		
		return path;
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if(start==null||goal==null) {
			throw new NullPointerException("Invalid start or goal");
		}
        VerticeNode S = vertices.get(start);
        VerticeNode G = vertices.get(goal);
        /*System.out.println(S.getPoint());
        System.out.println(G.getPoint());*/
        Map<VerticeNode,VerticeNode> parentMap = new HashMap<VerticeNode,VerticeNode>();
        //check if a path exists between start and goal using BFS search
        boolean found = BFSSearch(S,G,parentMap,nodeSearched);
        /*System.out.println("STATUS: "+found);
        System.out.println("Parent Map: "+parentMap);*/
        //if not found then returns a null
        if(!found) {
        	System.out.println("No Path between "+start+" and "+goal);
        	return null;
        }
        /*
         * A list containing the path of Vertice Nodes between
         * start and goal
         */
       
        List<GeographicPoint> path = constructPath(S,G,parentMap);
		return path;
	}
	private boolean dijkstraSearch(VerticeNode start,VerticeNode goal,
			Map<VerticeNode,VerticeNode> parentMap,Consumer<GeographicPoint> nodeSearched) {
		HashSet<VerticeNode> visited = new HashSet<VerticeNode>();
		PriorityQueue<VerticeNode> toExplore = new PriorityQueue<VerticeNode>(NumVertices, new VerticeNodeComparator());
		List<VerticeNode> distances = new ArrayList<VerticeNode>();
		
		for(VerticeNode vertice : vertices.values()) {
			vertice.setDistance(Double.POSITIVE_INFINITY);
			//vertice.setActualdistance(Double.POSITIVE_INFINITY);
		}
		start.setDistance(0.0);
		toExplore.add(start);
		boolean found = false;
		int count = 0;
		while(!toExplore.isEmpty()) {
			count++;
			VerticeNode curr = toExplore.remove();
			nodeSearched.accept(curr.getPoint());
			if(curr == goal) {
				found =true;
				break;
			}
			if(!visited.contains(curr)) {
				visited.add(curr);
				for(EdgeNode edge : curr.getNeighbors()) {
					VerticeNode neighbor= edge.getOtherEnd();
					
					if(!visited.contains(neighbor)) {
						double currDistance = curr.getDistance()+edge.getLength();
						if(currDistance < neighbor.getDistance()) {
							
							neighbor.setDistance(currDistance);
							parentMap.put(neighbor,curr);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}
	    //System.out.println("Dijkstra's Count: "+count);
		return found;
		
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
		if(start==null||goal==null) {
			throw new NullPointerException("Invalid start or goal");
		}
        VerticeNode S = vertices.get(start);
        VerticeNode G = vertices.get(goal);
        /*System.out.println(S.getPoint());
        System.out.println(G.getPoint());*/
        Map<VerticeNode,VerticeNode> parentMap = new HashMap<VerticeNode,VerticeNode>();
        boolean found = dijkstraSearch(S,G,parentMap,nodeSearched);
		return constructPath(S,G,parentMap);
	}
	
	private boolean aStarSearchAlgorithm(VerticeNode start,VerticeNode goal,
			Map<VerticeNode,VerticeNode> parentMap,Consumer<GeographicPoint> nodeSearched) {
		HashSet<VerticeNode> visited = new HashSet<VerticeNode>();
		PriorityQueue<VerticeNode> toExplore = new PriorityQueue<VerticeNode>(NumVertices, 
				new VerticeNodeComparator());
		List<VerticeNode> distances = new ArrayList<VerticeNode>();
		
		for(VerticeNode vertice : vertices.values()) {
			vertice.setActualdistance(Double.POSITIVE_INFINITY);
			vertice.setDistance(Double.POSITIVE_INFINITY);
		}
		start.setActualdistance(0.0);
		start.setDistance(0.0);
		toExplore.add(start);
		boolean found = false;
		int count = 0;
		while(!toExplore.isEmpty()) {
			count++;
			VerticeNode curr = toExplore.remove();
			nodeSearched.accept(curr.getPoint());
			if(curr == goal) {
				found =true;
				break;
			}
			if(!visited.contains(curr)) {
				visited.add(curr);
				for(EdgeNode edge : curr.getNeighbors()) {
					VerticeNode neighbor= edge.getOtherEnd();
					
					if(!visited.contains(neighbor)) {
						
						double currDist = edge.getLength()+curr.getActualdistance();
						// core of A* is just to add to currDist the cost of getting to
						// the destination
						double predDist = currDist+ (neighbor.getPoint()).distance(goal.getPoint());
						if(predDist < neighbor.getActualdistance()){
							// debug
							// System.out.println("Adding to queue node at: "+neighbor.getLocation());
							// System.out.println("Curr dist: "+currDist+" Pred Distance: " + predDist);
							
							parentMap.put(neighbor, curr);
							neighbor.setActualdistance(currDist);
							neighbor.setDistance(predDist);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}
	    //System.out.println("AStar Algorithm : "+count);
		return found;
		
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
        /*for(GeographicPoint point : vertices.keySet()) {
        	System.out.println(vertices.get(point));
        }*/
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
		if(start==null||goal==null) {
			throw new NullPointerException("Invalid start or goal");
		}
        VerticeNode S = vertices.get(start);
        VerticeNode G = vertices.get(goal);
        /*System.out.println(S.getPoint());
        System.out.println(G.getPoint());*/
        Map<VerticeNode,VerticeNode> parentMap = new HashMap<VerticeNode,VerticeNode>();
        boolean found = aStarSearchAlgorithm(S,G,parentMap,nodeSearched);
        //System.out.println("Boolean : "+found);
		return constructPath(S,G,parentMap);
		
	}
    public String toString() {
    	//Set<GeographicPoint> points = getVertices();
    	for(VerticeNode vertice : vertices.values()) {
    		System.out.println(vertice);
    	}
    	return "";
    }
	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		//System.out.println("Vertices: ");
		//firstMap.toString();
		
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		//List<GeographicPoint> testroute0 = simpleTestMap.bfs(testStart,testEnd);
		
		
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
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		
		
		
	}
	
}
