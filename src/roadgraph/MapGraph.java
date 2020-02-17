/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import basicgraph.Graph;
import com.sun.org.apache.xpath.internal.functions.FuncFalse;
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
	private Map<GeographicPoint, MapNode> graph;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		this.graph  = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return this.graph.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		List<GeographicPoint> v = new ArrayList<>();

		for(GeographicPoint g : graph.keySet()){
			v.add(this.graph.get(g).getLocation());
		}

		return new HashSet<GeographicPoint>(v);
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

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		int numEdges = 0;
		for(GeographicPoint g : graph.keySet()){
			numEdges += graph.get(g).getNumEdges();
		}

		return numEdges;
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
		if(location == null) return false;

		if(graph.containsKey(location)){
			return false;
		}


		graph.put(location, new MapNode(location));
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

		if(!getVertices().contains(from) || !getVertices().contains(to)){
			throw new IllegalArgumentException();
		}

		MapNode fromNode = getVertex(from);
		if (fromNode == null) throw new IllegalArgumentException();

		fromNode.addEdge(to, roadName,roadType, length);
		fromNode.addNeigbor(getVertex(to));
	}

	private MapNode getVertex(GeographicPoint location){
		if(graph.containsKey(location)){
			return graph.get(location);
		}
		return null;
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

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}

		MapNode startNode = getVertex(start);
		Queue<MapNode> queue = new LinkedList<>();
		queue.add(startNode);
		Set<MapNode> visited = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<>();

		if(found(queue, visited, parentMap, goal, nodeSearched)){
			return reconstructPath(parentMap, start, goal);
		}
		return new ArrayList<GeographicPoint>();
	}

	private List<GeographicPoint> reconstructPath(Map<MapNode, MapNode> parentMap,
												  GeographicPoint start,GeographicPoint endPoint){

		List<GeographicPoint> path = new ArrayList<>();
		GeographicPoint current = endPoint;

		while (current != null) {
			path.add(current);

			try {
				current = parentMap.get(getVertex(current)).getLocation();
			} catch (NullPointerException e) {
				break;
			}
		}

		Collections.reverse(path);

		return path;
	}

	private boolean found(Queue<MapNode> queue, Set<MapNode> visited,
						  Map<MapNode, MapNode> parent, GeographicPoint goal,
						  Consumer<GeographicPoint> nodeSearched){
		MapNode curr;

		while(!queue.isEmpty()){

			curr = queue.remove();
			nodeSearched.accept(curr.getLocation());

			if(curr.getLocation().equals(goal)){
				return true;
			}

			for(MapNode next : getNeighbors(curr.getLocation())) {
				if (!visited.contains(next)) {
					visited.add(curr);
					parent.put(next, curr);
					queue.add(next);
				}
			}
		}
		return false;
	}

	private List<MapNode> getNeighbors(GeographicPoint location){
		return this.graph.get(location).getNeighbors();
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

		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}

		PriorityQueue<MapNode> priorityQueue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		MapNode startNode = getVertex(start);

		try{
			startNode.setPriority(0);
		} catch (NullPointerException e) {
			startNode = new MapNode(start);
			startNode.setPriority(0);
		}

		priorityQueue.add(startNode);

		while(!priorityQueue.isEmpty()){
			MapNode curr = priorityQueue.remove();
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal)){
					return reconstructPath(parentMap, start, goal);
				}
				for(MapNode next : curr.getNeighbors()){
					if(!visited.contains(next)){
						double perhapPath = curr.getPriority() + curr.getLocation().distance(next.getLocation());
						if(perhapPath < next.getPriority()){
							// Hook for visualization.  See writeup.
							nodeSearched.accept(next.getLocation());
							next.setPriority(perhapPath);
							parentMap.put(next, curr);
							next.setPriority(perhapPath);
							priorityQueue.add(next);
						}
					}
				}
			}
		}

		return new ArrayList<>();
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

		PriorityQueue<MapNode> priorityQueue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		MapNode startNode = getVertex(start);

		try{
			startNode.setPriority(0);
		} catch (NullPointerException e) {
			startNode = new MapNode(start);
			startNode.setPriority(0);
		}

		priorityQueue.add(startNode);

		while(!priorityQueue.isEmpty()){
			MapNode curr = priorityQueue.remove();
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal)){
					return reconstructPath(parentMap, start, goal);
				}
				for(MapNode next : curr.getNeighbors()){
					if(!visited.contains(next)){
						double perhapPath = curr.getPriority() + curr.getLocation().distance(next.getLocation());
						double projection = perhapPath + next.getLocation().distance(goal);
						if(perhapPath < next.getProjection()){
							// Hook for visualization.  See writeup.
							nodeSearched.accept(next.getLocation());
							next.setPriority(perhapPath);
							parentMap.put(next, curr);
							next.setPriority(perhapPath);
							next.setProjection(projection);
							priorityQueue.add(next);
						}
					}
				}
			}
		}

		return new ArrayList<>();
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	/*
	Project Extension: Traveling salesperson problem
	Implementation using greedy algorithm
	*/
	public List<GeographicPoint> salespersonProblem(Set<GeographicPoint> vertices, GeographicPoint start){
		List<GeographicPoint> route = new ArrayList<>();
		route.add(start);
		PriorityQueue<MapNode> queue = new PriorityQueue<>();

		for (GeographicPoint node : vertices) {
			List<GeographicPoint> l = aStarSearch(getVertex(start).getLocation(), getVertex(node).getLocation());
			queue.add(getVertex(l.get(l.size() - 1)));
		}
		while(!queue.isEmpty()){
			route.add(queue.remove().getLocation());
		}

		route.add(start);
		return route;
	}


	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(7.0, 3.0);

		//Testing the extension
		Set<GeographicPoint> d = new HashSet<>();
		d.add(new GeographicPoint(7.0,  3.0));
		d.add(new GeographicPoint(4.0,  0.0));
		d.add(new GeographicPoint(5.0,  1.0));


		List<GeographicPoint> route = firstMap.salespersonProblem(d, start);
//		System.out.println(route);
		
		// You can use this method for testing.


		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */

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
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		/* Use this code in Week 3 End of Week Quiz */

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		start = new GeographicPoint(32.8648772, -117.2254046);
		end = new GeographicPoint(32.8660691, -117.217393);


		route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);


		
	}
	
}