package Tue;

import Tue.load.Display.Display;
import Tue.load.Forces.*;
import Tue.load.PointPlacement;
import Tue.objects.*;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;

import java.awt.*;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Main {

    public int width = 1200;
    public int height = 800;
    public int delta = 1;

    public ArrayList<Node> nodes = new ArrayList<Node>();
    public ArrayList<Edge> edges = new ArrayList<Edge>();

    public ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    public ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    public Force forces;
    public PointPlacement points;

    public double[][] clusterDistance;
    public double[][] pairD;

    public static void main(String[] args)
    {

        Main main = new Main();
        main.execute( main );

    }

    private void execute(final Main main ) {

        final DotParser parser = parserInput();

        nodes = parser.getNodes();
        edges = parser.getEdges();

        //this will overrule the parser and make hand defined nodes, edges and clusters
        //handMadeGraph();

        forces = new Force(width, height);

        points = new PointPlacement(nodes, edges, forces);
        points.PointPlacementCluster();

        clusterDistance = points.getClusterD();
        pairD = points.getPairD();

        clusters = points.getClusters();
        clusterEdges = points.getClusterEdges();

        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Display(main).create();
            }
        });
    }

    private DotParser parserInput()
    {
        DotScanner scanner = null;
        try {
            scanner = new DotScanner(new FileReader("datasets/universitiesclean.gv"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        DotParser parser = new DotParser(scanner, nodes, edges);
        try {
            parser.graph();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return parser;
    }

    private void handMadeGraph()
    {
        Node node1 = new Node("node1", 0);
        node1.addLabel("node1");
        Node node2 = new Node("node2", 1);
        node2.addLabel("node2");
        Node node3 = new Node("node3", 2);
        node3.addLabel("node3");

        node1.addWeight(10);
        node1.addCluster("1");
        node2.addWeight(10);
        node2.addCluster("2");
        node3.addWeight(10);
        node3.addCluster("3");
        nodes.clear();
        nodes.add(node1);
        nodes.add(node2);
        nodes.add(node3);

        Edge edge1 = new Edge(nodes.get(0), nodes.get(1));
        Edge edge2 = new Edge(nodes.get(1), nodes.get(2));
        Edge edge3 = new Edge(nodes.get(2), nodes.get(0));

        edges.clear();
        edges.add(edge1);
        edges.add(edge2);
        edges.add(edge3);
    }
}