package roadgraph;

import java.util.Comparator;

public class VerticeNodeComparator2 implements Comparator<VerticeNode> {
	public int compare(VerticeNode x1,VerticeNode x2) {
		if(x1.getActualdistance()>x2.getActualdistance())return 1;
		if(x1.getActualdistance() < x2.getActualdistance())return -1;
		return 0;
	}

}
