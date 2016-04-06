package Tue.load;

import Tue.load.Forces.Force;
import Tue.objects.*;
import mdsj.MDSJ;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * Created by s138362 on 6-4-2016.
 */
public class PointPlacement
{
    public ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    public ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    private double missingValue = 0;

    private int width = 1200;
    private int height = 800;

    public PointPlacement(ArrayList<Node> nodes, ArrayList<Edge> edges, Force forces)
    {
        Graph g = new Graph(nodes, edges);

        double[][] pairD = new double[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        int clusterNumber = getClusterNumber(nodes, edges);
        Cluster[] Cnodes = new Cluster[(clusterNumber+1)];

        double[][] clusterD = getDistanceMatrixCluster(nodes, clusterNumber, pairD);
        double[][] pos = defineNodePosition( clusterD );

        //define all cluster nodes
        for( int i = 0; i < clusterNumber; i++ )
        {
            Cnodes[i] = new Cluster( i, forces );
            Cnodes[i].setPos( new Vector2((float)pos[0][i], (float)pos[1][i] ));
            clusters.add(Cnodes[i]);
        }

        getClusterNodes(nodes);
        getClusterWeights(nodes);

        //define all cluster edges
        for( int i = 0; i < clusterNumber; i++ )
        {
            for( int j = (i+1); j < clusterNumber; j++ )
            {
                if( clusterD[i][j] != missingValue ) {
                    clusterEdges.add(new ClusterEdge(Cnodes[i], Cnodes[j], (clusterD[i][j] * 100), forces));
                }
            }
        }
    }

    private double[][] getDistanceMatrixCluster(ArrayList<Node> nodes, int clusterNumber, double[][] pairD)
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

        missingValue = highest*1.5;

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


    private int getClusterNumber(ArrayList<Node> nodes, ArrayList<Edge> edges)
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

    private void getClusterNodes(ArrayList<Node> nodes)
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

    private void getClusterWeights(ArrayList<Node> nodes)
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

    public ArrayList<Cluster> getClusters()
    {
        return clusters;
    }

    public ArrayList<ClusterEdge> getClusterEdges()
    {
        return clusterEdges;
    }
}
