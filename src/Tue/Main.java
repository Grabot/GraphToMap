package Tue;

import Tue.load.Display;
import Tue.load.Forces.*;
import Tue.load.PointPlacement;
import Tue.load.Vector2;
import Tue.objects.*;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;
import mdsj.MDSJ;

import java.awt.EventQueue;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

public class Main {

    public int width = 1200;
    public int height = 800;
    public int delta = 40;

    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    public ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    private double missingValue = 0;

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

        Force forces = new Force(width, height);

        PointPlacement points = new PointPlacement(nodes, edges, forces);

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
            scanner = new DotScanner(new FileReader("datasets/sample5.gv"));
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
        Node node4 = new Node("node4", 3);
        node4.addLabel("node4");
        Node node5 = new Node("node5", 4);
        node5.addLabel("node5");
        Node node6 = new Node("node6", 5);
        node6.addLabel("node6");
        Node node7 = new Node("node7", 6);
        node7.addLabel("node7");
        Node node8 = new Node("node8", 7);
        node8.addLabel("node8");
        Node node9 = new Node("node9", 8);
        node9.addLabel("node9");

        node1.addWeight(10);
        node1.addCluster("1");
        node2.addWeight(10);
        node2.addCluster("2");
        node3.addWeight(10);
        node3.addCluster("3");
        node4.addWeight(10);
        node4.addCluster("4");
        node5.addWeight(10);
        node5.addCluster("5");
        node6.addWeight(10);
        node6.addCluster("6");
        node7.addWeight(10);
        node7.addCluster("7");
        node8.addWeight(10);
        node8.addCluster("8");
        node9.addWeight(10);
        node9.addCluster("9");
        nodes.clear();
        nodes.add(node1);
        nodes.add(node2);
        nodes.add(node3);
        nodes.add(node4);
        nodes.add(node5);
        nodes.add(node6);
        nodes.add(node7);
        nodes.add(node8);
        nodes.add(node9);

        Edge edge1 = new Edge(nodes.get(0), nodes.get(1));
        Edge edge2 = new Edge(nodes.get(0), nodes.get(2));
        Edge edge3 = new Edge(nodes.get(0), nodes.get(3));
        Edge edge4 = new Edge(nodes.get(0), nodes.get(4));
        Edge edge5 = new Edge(nodes.get(0), nodes.get(5));
        Edge edge6 = new Edge(nodes.get(0), nodes.get(6));
        Edge edge7 = new Edge(nodes.get(0), nodes.get(7));
        Edge edge8 = new Edge(nodes.get(0), nodes.get(8));
        Edge edge9 = new Edge(nodes.get(0), nodes.get(9));

        edges.clear();
        edges.add(edge1);
        edges.add(edge2);
        edges.add(edge3);
        edges.add(edge4);
        edges.add(edge5);
        edges.add(edge6);
        edges.add(edge7);
        edges.add(edge8);
        edges.add(edge9);
    }
}