/*A VerticeNode which represents Geographic location 
 * of a point.
 * Parameters:
 *		point-represents longitutdes, latitudes
 *		neighbors- List of EdgeNodes from given point
*/
package roadgraph;
import java.util.*;
import geography.GeographicPoint;
public class VerticeNode {
	private GeographicPoint point;
	private List<EdgeNode> neighbors;
	private double distance;
	private double actualdistance;
	public VerticeNode() {
		point = null;
		neighbors = new ArrayList<EdgeNode>();
		setDistance(0.0);
		setActualdistance(0.0);
		
	}
	//constructor to initialize point with location
	public VerticeNode(GeographicPoint location) {
		point = location;
		neighbors = new ArrayList<EdgeNode>();
		setDistance(0.0);
		setActualdistance(0.0);
	}
	//returns copy of location
	public GeographicPoint getPoint() {
		return new GeographicPoint(this.point.getX(),this.point.getY());
	}
	//returns List of neighbors
	public List<EdgeNode> getNeighbors(){
		return new ArrayList<EdgeNode>(neighbors);
	}
	/*
	 * set the point value to location
	 * returns false if location is null or
	 * if values are equal
	 */
	public boolean setPoint(GeographicPoint location) {
		if((this.point.x == location.x && this.point.y ==location.y)|| location == null )
			return false;
		this.point = location;
		return true;
	}
	/*public boolean setNeigbors(List<GeographicPoint> newNeighbors) {
		int count = 0;
		for(GeographicPoint geo : neighbors) {
			if(newNeighbors.contains(geo)) {
				count++;
			}
		}
		if(count == neighbors.size() || newNeighbors == null)
			return false;
		this.neighbors = newNeighbors;
		return true;
	}*/
	/*
	 * adds details of neighbor to the neighbors list
	 * associated with current point.
	 * returns false if the neighbor already exists
	 */
	public boolean addNeighbor(VerticeNode newNeighbor,String roadName,String roadType
			,double length) {
		for(EdgeNode neighbor1 : neighbors) {
            //GeographicPoint neighbor = neighbor1.getOtherEnd().getPoint();
			if(neighbor1.getOtherEnd()==newNeighbor&&neighbor1.getRoadName().equals(roadName)&&neighbor1.getRoadType().equals(roadType)
					&& neighbor1.getLength() == length) {
				return false;
		    }
		}
		if(newNeighbor == null)return false;
		neighbors.add(new EdgeNode(newNeighbor,roadName,roadType,length));
		
		return true;
	}
    public String toString() {
    	System.out.println("("+point.getX()+","+point.getY()+")");
    	System.out.println("Neighbors :");
    	for(EdgeNode neighbor: neighbors) {
    		System.out.println("\tCoordinates : "+"("+neighbor.getOtherEnd().getPoint().getX()
    				+","+neighbor.getOtherEnd().getPoint().getY()+")");
    		System.out.println("\tRoad Name : "+neighbor.getRoadName());
    		System.out.println("\tRoad Type : "+neighbor.getRoadType());
    		System.out.println("\tLength : "+neighbor.getLength()+"\n");
    	}
    	return "";
    }
	public double getDistance() {
		return distance;
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}
	public double getActualdistance() {
		return actualdistance;
	}
	public void setActualdistance(double actualdistance) {
		this.actualdistance = actualdistance;
	}

}
