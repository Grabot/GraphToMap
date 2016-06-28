package Tue;

import Tue.load.Display.Display;
import Tue.load.Forces.*;
import Tue.load.Geometry.ConvexHull;
import Tue.load.PointPlacement;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.*;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;

import java.awt.*;
import java.io.*;
import java.util.ArrayList;

public class Main {

    public int width = 1200;
    public int height = 800;
    public int delta = 40;

    public ArrayList<Node> nodes = new ArrayList<Node>();
    public ArrayList<Edge> edges = new ArrayList<Edge>();

    public ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    public ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    public Force forces;
    public PointPlacement points;

    public double[][] clusterDistance;
    public double[][] pairD;

    public double graphScaling;
    private PolygonSimple boundingPolygon;

    public static void main(String[] args) throws FileNotFoundException {

        Main main = new Main();
        main.execute( main );

    }

    private void execute2( Main main )
    {
    }

    private void ellipseBorder()
    {
        boundingPolygon = new PolygonSimple();

        int numPoints = 20;
        for (int j = 0; j < numPoints; j++)
        {
            double angle = 2.0 * Math.PI * (j * 1.0 / numPoints);
            double rotate = 2.0 * Math.PI / numPoints / 2;
            double y = Math.sin(angle + rotate) * (height/2) + (height/2);
            double x = Math.cos(angle + rotate) * (width/2) + (width/2);
            boundingPolygon.add(x, y);
        }
    }

    private void writeArrayToFile(double[][] distanceMatrix, String fileName)
    {
        try {
            PrintStream output = new PrintStream(new File(fileName));
            for (int i = 0; i < distanceMatrix.length; i++) {
                for (int j = 0; j < distanceMatrix[i].length; j++) {
                    output.print("" + distanceMatrix[i][j] + ", ");
                }
                output.println("");
            }
        }
        catch( Exception e ) {
        }
    }

    private void execute(final Main main ) {

        forces = new Force(width, height);
        //RecipesWithClustering3
        //universitiesclean
        //miserables_3
        String fileName = "universitiesclean";
        final DotParser parser = parserInput(forces, fileName);

        nodes = parser.getNodes();
        edges = parser.getEdges();

        ellipseBorder();

        //this will overrule the parser and make hand defined nodes, edges and clusters
        //handMadeGraph();

        points = new PointPlacement(boundingPolygon, nodes, edges, forces);
        points.findPairDFile( "" + fileName + ".csv");
        //points.findPairDCalc();
        points.PointPlacementCluster();

        clusterDistance = points.getClusterD();
        pairD = points.getPairD();

        //writeArrayToFile( pairD, "" + fileName + ".csv");
        clusters = points.getClusters();
        clusterEdges = points.getClusterEdges();

        graphScaling = points.getScaling();

        for( Edge e : edges )
        {
            e.setWeight(e.getWeight()*(graphScaling/10));
        }

        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Display(main).create();
            }
        });

    }

    private DotParser parserInput(Force forces, String fileName)
    {
        DotScanner scanner = null;
        try {
            scanner = new DotScanner(new FileReader("datasets/" + fileName + ".gv"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        DotParser parser = new DotParser(scanner, nodes, edges, forces);
        try {
            parser.graph();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return parser;
    }

    private void handMadeGraph()
    {
        Node node1 = new Node(forces, "node1", 0);
        node1.addLabel("node1");
        Node node2 = new Node(forces, "node2", 1);
        node2.addLabel("node2");
        Node node3 = new Node(forces, "node3", 2);
        node3.addLabel("node3");

        Node node4 = new Node(forces, "node4", 3);
        node1.addLabel("node4");
        Node node5 = new Node(forces, "node5", 4);
        node2.addLabel("node5");
        Node node6 = new Node(forces, "node6", 5);
        node3.addLabel("node6");

        node1.addWeight(1);
        node1.addCluster("1");
        node2.addWeight(1);
        node2.addCluster("2");
        node3.addWeight(1);
        node3.addCluster("3");
        node4.addWeight(1);
        node4.addCluster("1");
        node5.addWeight(1);
        node5.addCluster("2");
        node6.addWeight(1);
        node6.addCluster("3");

        nodes.clear();
        nodes.add(node1);
        nodes.add(node2);
        nodes.add(node3);
        nodes.add(node4);
        nodes.add(node5);
        nodes.add(node6);

        Edge edge1 = new Edge(nodes.get(0), nodes.get(1), forces);
        Edge edge2 = new Edge(nodes.get(1), nodes.get(2), forces);
        Edge edge3 = new Edge(nodes.get(2), nodes.get(0), forces);

        Edge edge4 = new Edge(nodes.get(3), nodes.get(4), forces);
        Edge edge5 = new Edge(nodes.get(4), nodes.get(5), forces);
        Edge edge6 = new Edge(nodes.get(5), nodes.get(3), forces);

        Edge edge7 = new Edge(nodes.get(0), nodes.get(3), forces);
        Edge edge8 = new Edge(nodes.get(1), nodes.get(4), forces);
        Edge edge9 = new Edge(nodes.get(2), nodes.get(5), forces);

        edge1.setWeight(2);
        edge2.setWeight(2);
        edge3.setWeight(2);
        edge4.setWeight(2);
        edge5.setWeight(2);
        edge6.setWeight(2);
        edge7.setWeight(2);
        edge8.setWeight(2);
        edge9.setWeight(2);

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

    private void handMadeGraph2()
    {
        Node node1 = new Node(forces, "node1", 0);
        node1.addLabel("node1");
        Node node2 = new Node(forces, "node2", 1);
        node2.addLabel("node2");
        Node node3 = new Node(forces, "node3", 2);
        node3.addLabel("node3");

        node1.addWeight(1);
        node1.addCluster("1");
        node2.addWeight(1);
        node2.addCluster("2");
        node3.addWeight(1);
        node3.addCluster("3");

        nodes.clear();
        nodes.add(node1);
        nodes.add(node2);
        nodes.add(node3);

        Edge edge1 = new Edge(nodes.get(0), nodes.get(1), forces);
        Edge edge2 = new Edge(nodes.get(1), nodes.get(2), forces);
        Edge edge3 = new Edge(nodes.get(2), nodes.get(0), forces);

        edge1.setWeight(2);
        edge2.setWeight(2);
        edge3.setWeight(2);

        edges.clear();
        edges.add(edge1);
        edges.add(edge2);
        edges.add(edge3);
    }
}