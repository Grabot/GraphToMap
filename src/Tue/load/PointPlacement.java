package Tue.load;

import Tue.load.Forces.Force;
import Tue.objects.*;
import mdsj.MDSJ;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

/**
 * Created by s138362 on 6-4-2016.
 */
public class PointPlacement
{
    public ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    public ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<double[][]> positions = new ArrayList<double[][]>();

    private double missingValue = 0;
    private double radiusGlobal = 10;
    private double stepSize = 0.05;
    private int width = 1200;
    private int height = 800;

    private Random rand;
    private int amountnewpoints = 10;
    private int posamount = 1000;

    public PointPlacement(ArrayList<Node> nodes, ArrayList<Edge> edges, Force forces)
    {
        rand = new Random();

        Graph g = new Graph(nodes, edges);

        double[][] pairD = new double[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        int clusterNumber = getClusterNumber(nodes, edges);
        Cluster[] Cnodes = new Cluster[(clusterNumber+1)];

        double[][] clusterD = getDistanceMatrixCluster(nodes, clusterNumber, pairD);
        double[][] pos = defineNodePosition2( clusterD );

        distortionmetric( pos, clusterD );

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

    private void distortionmetric(double[][] pos, double[][] clusterD)
    {
        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        double[][] mapping = new double[clusterD.length][clusterD.length];

        for (int i = 0; i < clusterD.length; i++) {
            Vector2 node1 = new Vector2(pos[0][i], pos[1][i]);
            for (int j = 0; j < clusterD.length; j++)
            {
                Vector2 node2 = new Vector2(pos[0][j], pos[1][j]);
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[i][j] = node1.distance(node2);
            }
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            for( int j = 0; j < mapping[i].length; j++ )
            {
                double contractionlocal = (clusterD[i][j]/mapping[i][j]);
                double expansionlocal = (mapping[i][j]/clusterD[i][j]);

                if( contractionlocal > contraction )
                {
                    contraction = contractionlocal;
                }
                if( expansionlocal > expansion )
                {
                    expansion = expansionlocal;
                }
            }
        }
        distortion = (contraction*expansion);
        contraction = 0;
        expansion = 0;

        System.out.println("distortion: " + distortion);
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

    private Positioning[] p = new Positioning[100];
    private double[][] defineNodePosition2( double[][] clusterD )
    {
        for( int i = 0; i < p.length; i++ )
        {
            Positioning pos = new Positioning( null, Double.MAX_VALUE );
            p[i] = pos;
        }

        positions.clear();

        double[][] result = new double[clusterD.length][clusterD.length];

        for( int i = 0; i < posamount; i++ ) {
            double[][] output = new double[2][clusterD.length];
            for (int j = 0; j < clusterD.length; j++) {
                output[0][j] = (rand.nextDouble() * (width/2)) + 200;
                output[1][j] = (rand.nextDouble() * (height/2)) + 200;
            }
            positions.add(output);
        }


        while( radiusGlobal > 0.1 )
        {
            getDistortion(clusterD);
            radiusGlobal=(radiusGlobal-stepSize);
        }

        return p[99].getPos();
    }

    private void getDistortion( double[][] clusterD )
    {
        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        for( double[][] nodePos : positions )
        {
            double[][] mapping = new double[clusterD.length][clusterD.length];

            for (int i = 0; i < clusterD.length; i++) {
                Vector2 node1 = new Vector2(nodePos[0][i], nodePos[1][i]);
                for (int j = 0; j < clusterD.length; j++)
                {
                    Vector2 node2 = new Vector2(nodePos[0][j], nodePos[1][j]);
                    //get the actual distances between all nodes to calculate the distorion metrics
                    mapping[i][j] = node1.distance(node2);
                }
            }

            for( int i = 0; i < mapping.length; i++ )
            {
                for( int j = 0; j < mapping[i].length; j++ )
                {
                    double contractionlocal = (clusterD[i][j]/mapping[i][j]);
                    double expansionlocal = (mapping[i][j]/clusterD[i][j]);

                    if( contractionlocal > contraction )
                    {
                        contraction = contractionlocal;
                    }
                    if( expansionlocal > expansion )
                    {
                        expansion = expansionlocal;
                    }
                }
            }
            distortion = (contraction*expansion);
            contraction = 0;
            expansion = 0;

            if( p[0].getDistortion() > distortion )
            {
                Positioning pos = new Positioning( nodePos, distortion );
                p[0] = pos;
            }
            sort();
        }

        getNewPoints(clusterD.length);
    }

    private void getNewPoints( int matrixLength )
    {
        positions.clear();

        double radius;
        double degree;
        double x;
        double y;
        double newX;
        double newY;


        for( int i = 0; i < p.length; i++ )
        {
            for( int j = 0; j < amountnewpoints; j++ )
            {
                double[][] pos = p[i].getPos();
                double[][] newPos = new double[2][matrixLength];
                for( int k = 0; k < matrixLength; k++ )
                {
                    //get a random angle and a random length to find the next point placement.
                    degree = ((rand.nextDouble()*360)*(Math.PI / 180));
                    radius = rand.nextDouble()*radiusGlobal;
                    x = pos[0][k];
                    y = pos[1][k];
                    newX = (x + radius*Math.sin(degree));
                    newY = (y + radius*Math.cos(degree));
                    newPos[0][k] = newX;
                    newPos[1][k] = newY;

                }
                positions.add(newPos);
            }
        }
    }

    private void sort()
    {
        int n = p.length;
        Positioning temp;
        for (int v = 1; v < n; v++) {
            if (p[v - 1].getDistortion() < p[v].getDistortion()) {
                temp = p[v - 1];
                p[v - 1] = p[v];
                p[v] = temp;
            }
        }

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
