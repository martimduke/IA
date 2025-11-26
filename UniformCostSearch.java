package searchalgorithm;
import java.util.*;
import searchproblem.SearchProblem;

public class UniformCostSearch extends SearchAlgorithm {

	private GraphSearch graph;
	
	public UniformCostSearch(SearchProblem p) {
		Queue<Node> pQueue = new PriorityQueue<>(11, Comparator.comparingDouble(Node::getPathCost));
		graph = new GraphSearch(p,pQueue);
	}
	
	public Node searchSolution() {
		return graph.searchSolution();
	}
	
	public Map<String,Number> getMetrics() {
		return graph.getMetrics();
	}

}
