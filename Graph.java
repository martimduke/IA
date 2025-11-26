package undirectedgraph;

import java.util.*;

import searchalgorithm.*;
import searchproblem.SearchProblem;
import searchproblem.State;


public class Graph {
	private HashMap<String,Vertex> vertices;
	private HashMap<Integer,Edge> edges;	
	private ArrayList<VertexSet> vSets;
	
	private long expansions;
	private long generated;
	private long repeated;
	private double time;
	
	public Graph() {
		this.vertices = new HashMap<>();
		this.edges = new HashMap<>();
		this.vSets = new ArrayList<>();
		this.expansions = 0;
		this.generated = 0;
		this.repeated = 0;
		this.time = 0;
	}
	
	public void addVertex(String label, double lat, double lng) {
		Vertex v =  new Vertex(label);
		this.vertices.put(label, v);
		v.setCoordinates(lat, lng);
	}
	
	public Vertex getVertex(String label) {
		return this.vertices.get(label);
	}
	
	public void addVertexSet(String label) {
		VertexSet vSet =  new VertexSet(label);
		this.vSets.add(vSet);
	}
	
	public VertexSet getVertexSet(String setLabel) {
		for (VertexSet vSet : vSets) {
			if (Objects.equals(vSet.getLabel(), setLabel))
				return vSet;
		}
		return null;
	}
	
	public void addVertexToSet(String labelSet, String labelVertex) {
		Vertex v = this.vertices.get(labelVertex);
		for (VertexSet vSet : vSets) {
			if (Objects.equals(vSet.getLabel(), labelSet)) {
				vSet.addVertex(v);
				break;
			}
		}
	}
	
	public boolean addEdge(Vertex one, Vertex two, double weight) {
		if (two.equals(one)) return false;
		Edge e = new Edge(one,two,weight);
		if (edges.containsKey(e.hashcode())) return false;
		if (one.containsNeighbor(e) || two.containsNeighbor(e))  return false;	
		edges.put(e.hashcode(), e);
		one.addNeighbor(e);
		two.addNeighbor(e);
		return true;
	} 
	
	public boolean addEdge(String oneLabel, String twoLabel, double weight) {
		Vertex one = getVertex(oneLabel);
		Vertex two = getVertex(twoLabel);
		return addEdge(one,two,weight);
	}
	
	public boolean addEdge(String oneLabel, String twoLabel) {
		Vertex one = getVertex(oneLabel);
		Vertex two = getVertex(twoLabel);
		return addEdge(one,two,one.straightLineDistance(two));
	}

	public void showLinks() {
		System.out.println("********************* LINKS *********************");
		for (Vertex current: vertices.values()) {		
			System.out.print(current + ":" + " ");
			for (Edge e: current.getNeighbors()) {
				System.out.print(e.getNeighbor(current) + " (" + e.getWeight() + "); ");
			}
			System.out.println();
		}
		System.out.println("*************************************************");
	}
	
	public void showSets() {
		System.out.println("********************* SETS *********************");
		for (VertexSet vSet: vSets) {
			System.out.println(vSet);
		}
		System.out.println("*************************************************");
	}
	
	public Node searchSolution(String initLabel, String goalLabel, Algorithms algID) {
		State init = new State(this.getVertex(initLabel));
		State goal = new State(this.getVertex(goalLabel));
		SearchProblem prob = new SearchProblem(init,goal);
		SearchAlgorithm alg = null;
		switch (algID) {
			case BreadthFirstSearch -> alg = new BreadthFirstSearch(prob);
			case DepthFirstSearch -> alg = new DepthFirstSearch(prob);
			case UniformCostSearch -> alg = new UniformCostSearch(prob);
			case GreedySearch -> alg = new GreedySearch(prob);
			case AStarSearch -> alg = new AStarSearch(prob);
			default -> System.out.println("This algorithm is not implemented yet!");
		}
		Node n = alg.searchSolution();	
		Map<String,Number> m = alg.getMetrics();
		this.expansions += (long)m.get("Node Expansions");
		this.generated += (long)m.get("Nodes Generated");
		this.repeated += (long)m.get("State repetitions");
		this.time += (double)m.get("Runtime (ms)");
		return n;
	}

	public void showSolution(Node n) {
		System.out.println("******************* SOLUTION ********************");
		System.out.println("Node Expansions: " + this.expansions);
		System.out.println("Nodes Generated: " + this.generated);
		System.out.println("State Repetitions: " + this.repeated);
		System.out.printf("Runtime (ms): %6.3f \n",this.time);
		Node ni;
		List<Object> solution = n.getPath();
		double dist = 0;
		for (int i = 0; i<solution.size()-1;i++) {
			System.out.printf("| %-9s | %4.0f | ",solution.get(i), dist);
			ni = searchSolution(solution.get(i).toString(), solution.get(i+1).toString(), Algorithms.AStarSearch);
			System.out.print(ni.getPath());	
			System.out.println(" -> " + (int)ni.getPathCost());
			dist += ni.getPathCost();
		}
		System.out.printf("| %-9s | %4.0f | \n",solution.get(solution.size()-1), dist);
		System.out.println("*************************************************");
	}

    public Node searchSolutionProvince(String initLabel, String goalLabel, Algorithms algID, String provinceLabel) {
        Graph newG = new Graph();

        Vertex i = this.getVertex(initLabel);
        newG.addVertex(initLabel, i.getLatitude(), i.getLongitude());

        Vertex j = this.getVertex(goalLabel);
        newG.addVertex(goalLabel, j.getLatitude(), j.getLongitude());

        VertexSet p = this.getVertexSet(provinceLabel);
        HashSet<Vertex> pCidades = p.getVertices();
        for (Vertex pv : pCidades) {
            newG.addVertex(pv.getLabel(), pv.getLatitude(), pv.getLongitude());
        }

        for (Vertex pv : pCidades) {
            String via = pv.getLabel();

            Node n1 = this.searchSolution(initLabel, via, Algorithms.AStarSearch);
            double c1 = n1.getPathCost();

            Node n2 = this.searchSolution(via, goalLabel, Algorithms.AStarSearch);
            double c2 = n2.getPathCost();

            //caminhos
            newG.addEdge(initLabel, via, c1);
            newG.addEdge(via, goalLabel, c2);
        }

        Node solucao = newG.searchSolution(initLabel, goalLabel, algID);
        return solucao;

    }
    public Node searchSolution(String initLabel, String goalLabel, Algorithms algID, String... provinceLabel) {
        Graph newG = new Graph();
        Vertex i = this.getVertex(initLabel);
        Vertex j = this.getVertex(goalLabel);
        newG.addVertex(initLabel, i.getLatitude(), i.getLongitude());
        newG.addVertex(goalLabel, j.getLatitude(), j.getLongitude());

        //É uma estrutura para guardar por ordem as provincias e as suas cidades
        List<Set<Vertex>> chain = new ArrayList<>();


        for(String NomeProvincia : provinceLabel) {
            VertexSet vs = this.getVertexSet(NomeProvincia); //Procura o nome das cidades dessa provincia
            Set<Vertex> pCidades = vs.getVertices(); //Coloca em pCidades todas as cidades dessa provincia
            chain.add(pCidades); //Adiciona a lista esse conjunto de cidades dessa provincia

            for(Vertex v : pCidades) {  //Percorrer cada cidade da provincia
                newG.addVertex(v.getLabel(), v.getLatitude(), v.getLongitude());  //E adicionar ao newG as suas coordenadas e nome
            }
        }
        // Criar os caminhos da cidade inicial para todas as ciadades da provincia 1
        for(Vertex v1 : chain.get(0)) {   //v1 é uma cidade do conjunto de cidades da provincia chain(0)
            Node n = this.searchSolution(initLabel, v1.getLabel(), algID);  //criamos o no n para guardar a informaçao da cidade 1 ate v1 e o melhor caminho
            newG.addEdge(initLabel, v1.getLabel(), n.getPathCost());   //cria o caminho da ciade 1 ate v1
        }

        // criar os caminhos da provincia 1 ate a proxima provincia
        for(int m=0; m< chain.size()-1;m++){
            for(Vertex a : chain.get(m)) {
                for(Vertex b : chain.get(m+1)) {
                    Node n= this.searchSolution(a.getLabel(), b.getLabel(), algID);
                    newG.addEdge(a.getLabel(), b.getLabel(), n.getPathCost());
                }
            }
        }

        //Ligar a ultima ciadade da ultima provincia escolhida a cidade final
        Set<Vertex> ultima= chain.get(chain.size()-1);   //ultima fica com todas as cidades da ultima provincia
        for(Vertex vUltima : ultima) {
            Node n = this.searchSolution(vUltima.getLabel(), goalLabel, algID);
            newG.addEdge(vUltima.getLabel(),goalLabel,n.getPathCost());
        }


        return newG.searchSolution(initLabel, goalLabel, algID);  //faz o caminho final da cidade 1 ate a final a passar pelo melhor caminho de cada cidade de cada provincia

    }

    }
