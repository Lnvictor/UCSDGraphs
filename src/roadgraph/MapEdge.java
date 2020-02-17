package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
    private GeographicPoint from;
    private GeographicPoint to;
    private String roadName;
    private String roadType;
    private double lenght;

    public MapEdge(GeographicPoint from, GeographicPoint to,
                   String roadName, String roadType, double length){

        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.lenght = length;
    }

    public  GeographicPoint getTo(){
        return to;
    }

    public GeographicPoint getFrom(){ return from; };

    public  double getDistance(){
        return this.lenght;
    }

    public String getRoadName(){
        return this.roadName;
    }

    public String getRoadType(){
        return this.roadType;
    }
}
