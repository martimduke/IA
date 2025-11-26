package undirectedgraph;
import java.util.*;

public class VertexSet {
	private HashSet<Vertex> vertices;
	private String label;
	
	public VertexSet(String label) {
		this.vertices = new HashSet<>();
		this.label = label;
	}
	
	public void addVertex(Vertex v) {
		this.vertices.add(v);
	}

	public HashSet<Vertex> getVertices() {
		return this.vertices;
	}
	
	public String getLabel() {
		return this.label;
	}
	
	public String toString() {
		StringBuilder s = new StringBuilder(label + ": ");
		for (Vertex v: vertices) s.append(v.toString()).append("; ");
		return s.toString();
	}
}
