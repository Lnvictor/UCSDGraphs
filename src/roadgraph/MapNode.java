package roadgraph;

import com.sun.org.apache.regexp.internal.RE;
import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class MapNode implements Comparable<MapNode>{
    private GeographicPoint location;
    private List<MapEdge> mapEdges;
    private List<MapNode> neigbors;
    private double priority;
    private double projection;

    public MapNode(GeographicPoint location){
        this.location = location;
        this.mapEdges = new ArrayList<>();
        this.neigbors = new ArrayList<>();
        this.priority = Double.POSITIVE_INFINITY;
        this.projection = Double.POSITIVE_INFINITY;
    }

    public GeographicPoint getLocation(){
        return this.location;
    }

    public boolean addEdge(GeographicPoint to, String roadName,
                   String roadType, double length){

        if(to == null || roadName == null|| roadType == null){
            return false;
        }

        this.mapEdges.add(new MapEdge(this.location, to, roadName, roadType, length));
        return true;
    }

    public boolean addNeigbor(MapNode node){
        if(node == null) return false;

        this.neigbors.add(node);
        return true;
    }

    public int getNumEdges(){
        return this.mapEdges.size();
    }

    public List<MapNode> getNeighbors() {
        return this.neigbors;
    }

    @Override
    public int compareTo(MapNode o) {
        if(this.priority > o.getPriority()){
            return 1;
        }
        if(this.priority < o.getPriority()){
            return -1;
        }
        return 0;
    }

    public void setPriority(double priority){
        this.priority = priority;
    }

    public double getPriority(){
        return this.priority;
    }

    public void setProjection(double projection){
        this.projection = projection;
    }

    public double getProjection(){
        return this.projection;
    }
}
