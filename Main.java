import searchalgorithm.Algorithms;
import searchalgorithm.Node;
import undirectedgraph.Graph;
import undirectedgraph.Romenia;


public class Main {
    public static void main(String[] args) {
        Graph graph = Romenia.defineGraph();
        graph.showLinks();
        graph.showSets();
        Node n;
        n = graph.searchSolution("Arad", "Bucharest", Algorithms.AStarSearch, "Dobrogea","Banat");
        graph.showSolution(n);
    }
}

