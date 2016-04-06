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
            scanner = new DotScanner(new FileReader("datasets/universities.gv"));
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

}