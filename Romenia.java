package undirectedgraph;

public class Romenia {

    public static Graph defineGraph() {
        
        Graph g = new Graph();
        // Define cities:
        g.addVertex("Arad", 46.18333, 21.31667);
        g.addVertex("Bucharest", 44.43225, 26.10626);
        g.addVertex("Craiova", 44.33018, 23.79488);
        g.addVertex("Dobreta", 44.63692, 22.65973);
        g.addVertex("Eforie", 44.05842, 28.63361);
        g.addVertex("Fagaras", 45.84164, 24.97310);
        g.addVertex("Giurgiu", 43.90371, 25.96993);
        g.addVertex("Hirsova", 44.68935, 27.94566);
        g.addVertex("Iasi", 47.15845, 27.60144);
        g.addVertex("Lugoj", 45.69099, 21.90346);
        g.addVertex("Mehadia", 44.90411, 22.36452);
        g.addVertex("Neamt", 46.97587, 26.38188);
        g.addVertex("Oradea", 47.04650, 21.91894);
        g.addVertex("Pitesti", 44.85648, 24.86918);
        g.addVertex("R. Vilcea", 45.09968, 24.36932);
        g.addVertex("Sibiu", 45.79833, 24.12558);
        g.addVertex("Timisoara", 45.74887, 21.20868);
        g.addVertex("Urziceni", 44.71653, 26.64112);
        g.addVertex("Vaslui", 46.64069, 27.72765);
        g.addVertex("Zerind", 46.62251, 21.51742);
        // Define connections:
        g.addEdge("Arad","Sibiu");
        g.addEdge("Arad","Timisoara");
        g.addEdge("Arad","Zerind");
        g.addEdge("Bucharest","Fagaras");
        g.addEdge("Bucharest","Giurgiu");
        g.addEdge("Bucharest","Pitesti");
        g.addEdge("Bucharest","Urziceni");
        g.addEdge("Craiova","Dobreta");
        g.addEdge("Craiova","Pitesti");
        g.addEdge("Craiova","R. Vilcea");
        g.addEdge("Dobreta","Mehadia");
        g.addEdge("Eforie","Hirsova");
        g.addEdge("Fagaras","Sibiu");
        g.addEdge("Hirsova","Urziceni");
        g.addEdge("Iasi","Neamt");
        g.addEdge("Iasi","Vaslui");
        g.addEdge("Lugoj","Mehadia");
        g.addEdge("Lugoj","Timisoara");
        g.addEdge("Oradea","Sibiu");
        g.addEdge("Oradea","Zerind");
        g.addEdge("Pitesti","R. Vilcea");
        g.addEdge("R. Vilcea","Sibiu");
        g.addEdge("Urziceni","Vaslui");
        // Define regions:
        g.addVertexSet("Banat");
        g.addVertexToSet("Banat","Lugoj");
        g.addVertexToSet("Banat","Mehadia");
        g.addVertexToSet("Banat","Timisoara");
        g.addVertexSet("Crisana");
        g.addVertexToSet("Crisana","Arad");
        g.addVertexToSet("Crisana","Oradea");
        g.addVertexToSet("Crisana","Zerind");
        g.addVertexSet("Dobrogea");
        g.addVertexToSet("Dobrogea","Eforie");
        g.addVertexToSet("Dobrogea","Hirsova");
        g.addVertexSet("Moldova");
        g.addVertexToSet("Moldova","Iasi");
        g.addVertexToSet("Moldova","Neamt");
        g.addVertexToSet("Moldova","Vaslui");
        g.addVertexSet("Muntenia");
        g.addVertexToSet("Muntenia","Bucharest");
        g.addVertexToSet("Muntenia","Giurgiu");
        g.addVertexToSet("Muntenia","Pitesti");
        g.addVertexToSet("Muntenia","Urziceni");
        g.addVertexSet("Oltenia");
        g.addVertexToSet("Oltenia","Craiova");
        g.addVertexToSet("Oltenia","Dobreta");
        g.addVertexToSet("Oltenia","R. Vilcea");
        g.addVertexSet("Transilvania");
        g.addVertexToSet("Transilvania","Fagaras");
        g.addVertexToSet("Transilvania","Sibiu");

        return g;
    }

}
