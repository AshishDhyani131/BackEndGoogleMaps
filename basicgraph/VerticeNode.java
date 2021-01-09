package basicgraph;
import java.util.*;
import geography.GeographicPoint;
public class VerticeNode {
	private GeographicPoint point;
	private List<EdgeNode> neighbors;
	public VerticeNode() {
		point = null;
		neighbors = new ArrayList<EdgeNode>();
	}
	public VerticeNode(GeographicPoint location) {
		point = location;
		neighbors = new ArrayList<EdgeNode>();
	}
	public GeographicPoint getPoint() {
		return new GeographicPoint(this.point.getX(),this.point.getY());
	}
	public List<EdgeNode> getNeighbors(){
		return new ArrayList<EdgeNode>(neighbors);
	}
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
}
