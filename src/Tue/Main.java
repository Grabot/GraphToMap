package Tue;

import Tue.load.Display;
import Tue.load.Forces.FrictionForce;
import Tue.load.Forces.SpringForce;
import Tue.load.Forces.WallForce;
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

    public ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    public ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();

    private SpringForce spring;
    private WallForce wall;
    private FrictionForce friction;

    public static void main(String[] args)
    {

        Main main = new Main();
        main.execute( main );

    }

    private void execute(final Main main ) {
        final DotParser parser = parserInput();

        nodes = parser.getNodes();
        edges = parser.getEdges();

        spring = new SpringForce();
        wall = new WallForce( width, height, delta );
        friction = new FrictionForce();

        Graph g = new Graph(nodes, edges);

        int clusterNumber = getClusterNumber();

        float[][] pairD = new float[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        double[][] clusterD = new double[clusterNumber][clusterNumber];
        ClusterNode[] Cnodes = new ClusterNode[(clusterNumber+1)];
        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = (i+1); j < clusterNumber; j++ )
            {
                ArrayList<Node> from = new ArrayList<Node>();
                ArrayList<Node> to = new ArrayList<Node>();
                for( Node node : nodes )
                {
                    if( node.getClusterNumber() == i )
                    {
                        from.add(node);
                    }
                    if( node.getClusterNumber() == j )
                    {
                        to.add(node);
                    }
                }
                float total = 0;
                for( Node f : from )
                {
                    for( Node t : to )
                    {
                        total = (total + pairD[f.getIndex()][t.getIndex()]);
                    }
                }
                if( total < 0 )
                {
                    clusterD[i][j] = -1;
                    clusterD[j][i] = -1;
                }
                else {
                    clusterD[i][j] = (total / (from.size() * to.size()));
                    clusterD[j][i] = (total / (from.size() * to.size()));
                }
                from.clear();
                to.clear();
            }
            clusterD[i][i] = 0;
        }

        double[][] pos = defineNodePosition( clusterD );

        //define all cluster nodes
        for( int i = 0; i < clusterNumber; i++ )
        {
            Cnodes[i] = new ClusterNode( i, wall, friction );
            Cnodes[i].setPos( new Vector2((float)pos[0][i], (float)pos[1][i] ));
            clusternodes.add(Cnodes[i]);
        }

        //define all cluster edges
        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = (i+1); j < clusterNumber; j++ )
            {
                clusteredges.add(new ClusterEdge(Cnodes[i], Cnodes[j], (clusterD[i][j]*100), spring));
            }
        }

//        for( int i = 0; i < clusterNumber; i++ )
//        {
//            for( int j = 0; j < clusterNumber; j++ )
//            {
//                System.out.print(clusterD[i][j] + " ");
//            }
//            System.out.println("");
//        }

        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Display(main).create();
            }
        });
    }

    private double[][] defineNodePosition(double[][] ClusterD)
    {

        //keep track of the highest number to define empty aspects of the distance matrix
        //there are "-1" distances, these mean no connection. We connect them with a factor of the highest distance
        //This will draw them in the plane away from the others, which is also what should happen
        double highest = 0;
        for( int i = 0; i < ClusterD.length; i++ )
        {
            for( int j = 0; j < ClusterD[i].length; j++ )
            {
                if( ClusterD[i][j] > highest )
                {
                    highest = ClusterD[i][j];
                }
            }
        }

        for( int i = 0; i < ClusterD.length; i++ )
        {
            for( int j = 0; j < ClusterD[i].length; j++ )
            {
                if( ClusterD[i][j] == -1 )
                {
                    ClusterD[i][j] = highest*1.4;
                }
            }
        }

        double[][] output= MDSJ.classicalScaling(ClusterD); // apply MDS

        for(int i=0; i<output[0].length; i++) {
            //coordinates are determined, scale them up to fit the plane
            output[0][i] = (output[0][i]*100) + (width/2);
            output[1][i] = (output[1][i]*100) + (height/2);
        }
        return output;
    }

    private int getClusterNumber()
    {
        HashSet<String> clusters = new HashSet<String>();

        for( Node node : nodes )
        {
            clusters.add(node.getCluster());
        }

        int clusterNumber = 0;

        for( String cluster : clusters )
        {
            for( Node node : nodes )
            {
                if( node.getCluster().equals(cluster))
                {
                    node.addClusterNumber(clusterNumber);
                }
            }
            clusterNumber++;
        }

        return clusterNumber;
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