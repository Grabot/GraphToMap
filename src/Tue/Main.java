package Tue;

import Tue.load.Display;
import Tue.load.SpringForce;
import Tue.load.Vector2;
import Tue.objects.*;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;

import java.awt.EventQueue;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

public class Main {

    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    public ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();

    private SpringForce spring;

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

        Graph g = new Graph(nodes, edges);

        int clusterNumber = getClusterNumber();

        float[][] pairD = new float[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        float[][] clusterD = new float[clusterNumber][clusterNumber];
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

        //define all cluster nodes
        for( int i = 0; i < clusterNumber; i++ )
        {
            Cnodes[i] = new ClusterNode( i );
        }

        //define all cluster edges
        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = (i+1); j < clusterNumber; j++ )
            {
                if( clusterD[i][j] != -1 ) {
                    clusteredges.add(new ClusterEdge(Cnodes[i], Cnodes[j], (clusterD[i][j]*100), spring));
                }
            }
        }

        //set random positions for cluster nodes.
        for( int i = 0; i < clusterNumber; i++ )
        {
            Cnodes[i].setPos(new Vector2( (float)(Math.random()*1200), (float)(Math.random()*800)));
            clusternodes.add(Cnodes[i]);
        }


        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = 0; j < clusterNumber; j++ )
            {
                System.out.print( clusterD[i][j] + " " );
            }
            System.out.println("");
        }

        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Display(main).create();
            }
        });
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
            scanner = new DotScanner(new FileReader("datasets/sample2.gv"));
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