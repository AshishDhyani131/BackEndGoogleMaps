package roadgraph;
import geography.GeographicPoint;
import java.util.*;
/*
 * stores the details of an edge 
 * associated with a point
 * Parameters:
 * 		otherEnd-represents the neighbor
 * 		roadName-name of road connecting two points
 * 		roadType-type of road
 * 		length-distance between two points 
 */
public class EdgeNode {
	private VerticeNode otherEnd;
	private String roadName;
	private String roadType;
	private double length;
	public EdgeNode() {
		setOtherEnd(new VerticeNode());
		setRoadName("");
		setRoadType("");
		setLength(0.0);
	}
	//constructor 
	public EdgeNode(VerticeNode other,String name,String type,double len) {
		setOtherEnd(other);
		setRoadName(name);
		setRoadType(type);
		setLength(len);
	}
	
	public VerticeNode getOtherEnd() {
		return otherEnd;
	}
	public void setOtherEnd(VerticeNode otherEnd) {
		this.otherEnd = otherEnd;
	}
	public String getRoadName() {
		return roadName;
	}
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	public String getRoadType() {
		return roadType;
	}
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	public double getLength() {
		return length;
	}
	public void setLength(double length) {
		this.length = length;
	}

}
