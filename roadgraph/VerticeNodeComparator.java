package roadgraph;

import java.util.Comparator;

public class VerticeNodeComparator implements Comparator<VerticeNode> {
	public int compare(VerticeNode x1,VerticeNode x2) {
		if(x1.getDistance()>x2.getDistance())return 1;
		if(x1.getDistance() < x2.getDistance())return -1;
		return 0;
	}

}
