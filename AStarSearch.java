package searchalgorithm;
import java.util.*;
import searchproblem.SearchProblem;
import searchproblem.State;

public class AStarSearch extends SearchAlgorithm {
	
	private GraphSearch graph;

	public AStarSearch(final SearchProblem p) {
		Queue<Node> pQueue = new PriorityQueue<>(101, (o1, o2) -> {
		State n1 = o1.getState();
		State n2 = o2.getState();
			return Double.compare(o1.getPathCost() + p.heuristic(n1), o2.getPathCost() + p.heuristic(n2));
		});
		graph = new GraphSearch(p,pQueue);
	}

	public Node searchSolution() {
		return graph.searchSolution();
	}
	
	public Map<String,Number> getMetrics() {
		return graph.getMetrics();
	}

}
