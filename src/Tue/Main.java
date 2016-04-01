package Tue;

import Tue.load.Display;
import Tue.load.Forces.CoulombForce;
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

    public ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
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

        SpringForce spring = new SpringForce();
        WallForce wall = new WallForce(width, height);
        FrictionForce friction = new FrictionForce();
        CoulombForce coulomb = new CoulombForce();

        Graph g = new Graph(nodes, edges);

        //set the correct cluster for every node and get the total number of clusters

        double[][] pairD = new double[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        //index 71 and 72 should be the disjointed nodes with universities dataset.


//        for( int i = 0; i < pairD.length; i++ )
//        {
//            for( int j = 0; j < pairD[i].length; j++ )
//            {
//                System.out.print( pairD[i][j] + " ");
//            }
//            System.out.println("");
//        }

        int clusterNumber = getClusterNumber();
        Cluster[] Cnodes = new Cluster[(clusterNumber+1)];

        double[][] clusterD = getDistanceMatrixCluster(clusterNumber, pairD);

        double[][] pos = defineNodePosition( clusterD );

        //define all cluster nodes
        for( int i = 0; i < clusterNumber; i++ )
        {
            Cnodes[i] = new Cluster( i, wall, friction, coulomb);
            Cnodes[i].setPos( new Vector2((float)pos[0][i], (float)pos[1][i] ));
            clusters.add(Cnodes[i]);
        }

        getClusterNodes();
        getClusterWeights();

        //define all cluster edges
        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = (i+1); j < clusterNumber; j++ )
            {
                if( clusterD[i][j] != missingValue ) {
                    clusteredges.add(new ClusterEdge(Cnodes[i], Cnodes[j], (clusterD[i][j] * 100), spring));
                }
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

    private double[][] getDistanceMatrixCluster(int clusterNumber, double[][] pairD)
    {
        double[][] clusterD = new double[clusterNumber][clusterNumber];

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
                double total = 0;
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
                else
                {
                    clusterD[i][j] = (total / (from.size() * to.size()));
                    clusterD[j][i] = (total / (from.size() * to.size()));
                }
                from.clear();
                to.clear();
            }
            clusterD[i][i] = 0;
        }

        return clusterD;
    }

    private double[][] defineNodePosition(double[][] clusterD)
    {

        //keep track of the highest number to define empty aspects of the distance matrix
        //there are "-1" distances, these mean no connection. We connect them with a factor of the highest distance
        //This will draw them in the plane away from the others, which is also what should happen
        double highest = 0;
        for( int i = 0; i < clusterD.length; i++ )
        {
            for( int j = 0; j < clusterD[i].length; j++ )
            {
                if( clusterD[i][j] > highest )
                {
                    highest = clusterD[i][j];
                }
            }
        }

        missingValue = highest*1.2;

        for( int i = 0; i < clusterD.length; i++ )
        {
            for( int j = 0; j < clusterD[i].length; j++ )
            {
                if( clusterD[i][j] == -1 )
                {
                    clusterD[i][j] = missingValue;
                }
            }
        }

        double[][] output = MDSJ.stressMinimization(clusterD);
        //double[][] output = MDSJ.classicalScaling(clusterD); // apply MDS

        for(int i=0; i<output[0].length; i++) {
            //coordinates are determined, scale them up to fit the plane
            output[0][i] = (output[0][i]*100) + (width/2);
            output[1][i] = (output[1][i]*100) + (height/2);
        }
        return output;
    }


    private double[][] removeData(double[][] source, int row, int column)
    {
        double[][] destinationarr = new double[(source.length-1)][(source.length-1)];

        int p = 0;
        for( int i = 0; i < source.length; ++i)
        {
            if ( i == row)
                continue;


            int q = 0;
            for( int j = 0; j < source[i].length; ++j)
            {
                if ( j == column)
                    continue;

                destinationarr[p][q] = source[i][j];
                ++q;
            }
            ++p;
        }

        return destinationarr;
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

    private void getClusterNodes()
    {
        for( Node node : nodes )
        {
            for( int i = 0; i < clusters.size(); i++ )
            {
                if( node.getClusterNumber() == i ) {
                    clusters.get(i).addNode(node);
                }
            }
        }
    }

    private void getClusterWeights()
    {
        int clusternumber = -1;
        double total = 0;
        for( Node node : nodes )
        {
            clusternumber = node.getClusterNumber();
            clusters.get(clusternumber).addWeight(node.getWeight());
            total = (total+node.getWeight());
        }

        for( Cluster cluster : clusters )
        {
            cluster.setPercentage(cluster.getWeight()/total);
        }
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