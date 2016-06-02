package Tue.load;

import Tue.load.Forces.Force;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.*;

import java.awt.*;
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
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private ArrayList<double[][]> positions = new ArrayList<double[][]>();
    private Force forces;

    private double missingValue = 0;
    private double radiusGlobal = 20;
    private double stepSize = 0.1;
    private int width = 1200;
    private int height = 800;

    private Random rand;
    private int amountnewpoints = 10;
    private int posamount = 1000;

    private double[][] clusterDistance;
    private double[][] pairD;
    private double graphScaling = 0;

    private PolygonSimple boundingPolygon;

    public PointPlacement(PolygonSimple boundingPolygon, ArrayList<Node> nodes, ArrayList<Edge> edges, Force forces )
    {
        rand = new Random();
        this.boundingPolygon = boundingPolygon;
        this.forces = forces;
        this.nodes = nodes;
        this.edges = edges;

        Graph g = new Graph(nodes, edges);

        pairD = new double[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }
    }

    public double getScaling()
    {
        return graphScaling;
    }

    public void PointPlacementCluster()
    {
        double graphDistance = 1;

        int clusterNumber = getClusterNumber(nodes, edges);
        Cluster[] Cnodes = new Cluster[(clusterNumber+1)];

        double[][] clusterD = getDistanceMatrixCluster(nodes, clusterNumber);
        clusterDistance = clusterD;

        for( int i = 0; i < clusterD.length; i++ )
        {
            for( int j = 0; j < clusterD[i].length; j++ )
            {
                if( clusterD[i][j] > graphDistance )
                {
                    graphDistance = clusterD[i][j];
                }
            }
        }

        graphScaling = (Math.sqrt((Math.pow(width, 2) + Math.pow(height, 2)))/graphDistance);
        System.out.println("scaling: " + graphScaling );

        double[][] pos = defineClusterNodePosition( clusterD );

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

    public double[][] getClusterD()
    {
        return clusterDistance;
    }

    public double[][] getPairD() { return pairD; }

    private double[][] getDistanceMatrixCluster(ArrayList<Node> nodes, int clusterNumber)
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

    private Positioning[] clusterP = new Positioning[100];
    private double[][] defineClusterNodePosition( double[][] clusterD )
    {
        double[][] empty = null;
        for( int i = 0; i < clusterP.length; i++ )
        {
            Positioning pos = new Positioning( empty, Double.MAX_VALUE );
            clusterP[i] = pos;
        }

        positions.clear();

        double Xpos = 0;
        double Ypos = 0;
        for( int i = 0; i < posamount; i++ ) {
            double[][] output = new double[2][clusterD.length];
            for (int j = 0; j < clusterD.length; j++) {
                Xpos = (rand.nextDouble() * width);
                Ypos = (rand.nextDouble() * height);
                output[0][j] = Xpos;
                output[1][j] = Ypos;
            }
            positions.add(output);
        }


        while( radiusGlobal > 0.1 )
        {
            getDistortionCluster(clusterD);
            radiusGlobal=(radiusGlobal-stepSize);
        }

        return clusterP[99].getClusterPos();
    }


    private void getDistortionCluster( double[][] clusterD )
    {

        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion = 1;

        double contractiontotal = 0;
        double expansiontotal = 0;
        double contractionlocal;
        double expansionlocal;
        int total = 0;
        int totalContraction = 0;
        int totalExpansion = 0;
        double contraction_expansion = 0;

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
                    if( i != j )
                    {
                        total++;
                        contractionlocal = (graphScaling*clusterD[i][j] / mapping[i][j]);
                        expansionlocal = (mapping[i][j] / graphScaling*clusterD[i][j]);

                        if( contractionlocal >= expansionlocal )
                        {
                            totalContraction++;
                            contractiontotal = (contractiontotal + contractionlocal);
                        }
                        else
                        {
                            totalExpansion++;
                            expansiontotal = (expansiontotal + expansionlocal);
                        }
                    }
                }
            }

            contraction_expansion = (contractiontotal + expansiontotal);
            distortion = (contraction_expansion/total);

//            contraction = (contractiontotal/total);
//            expansion = (expansiontotal/total);
//            if( contraction >= expansion )
//            {
//                distortion = contraction;
//            }
//            else
//            {
//                distortion = expansion;
//            }
//
//            contraction = 0;
//            expansion = 0;
            contractiontotal = 0;
            expansiontotal = 0;
            total = 0;
            totalContraction = 0;
            totalExpansion = 0;

            if( clusterP[0].getDistortion() > distortion )
            {
                Positioning pos = new Positioning( nodePos, distortion );
                clusterP[0] = pos;
            }
            sortCluster();
        }

        getNewPointsCluster(clusterD.length);
    }

    private void getNewPointsCluster( int matrixLength )
    {
        positions.clear();

        double radius;
        double degree;
        double x;
        double y;
        double newX;
        double newY;


        for( int i = 0; i < clusterP.length; i++ )
        {
            for( int j = 0; j < amountnewpoints; j++ )
            {
                double[][] pos = clusterP[i].getClusterPos();
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


    private void sortCluster()
    {
        int n = clusterP.length;
        Positioning temp;
        for (int v = 1; v < n; v++) {
            if (clusterP[v - 1].getDistortion() < clusterP[v].getDistortion()) {
                temp = clusterP[v - 1];
                clusterP[v - 1] = clusterP[v];
                clusterP[v] = temp;
            }
        }

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


    private void distortionmetric(double[][] pos, double[][] clusterD)
    {
        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        double contractiontotal = 0;
        double expansiontotal = 0;
        int total = 0;

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
                if( i != j )
                {
                    total++;
                    double contractionlocal = (clusterD[i][j] / mapping[i][j]);
                    double expansionlocal = (mapping[i][j] / clusterD[i][j]);

                    contractiontotal = (contractiontotal + contractionlocal);
                    expansiontotal = (expansiontotal + expansionlocal);
                }
            }
        }

        contraction = (contractiontotal/total);
        expansion = (expansiontotal/total);
        distortion = (contraction*expansion);

        contraction = 0;
        expansion = 0;
        contractiontotal = 0;
        expansiontotal = 0;
        total = 0;

        System.out.println("distortion: " + distortion);
    }


    public ArrayList<Cluster> getClusters()
    {
        return clusters;
    }

    public ArrayList<ClusterEdge> getClusterEdges()
    {
        return clusterEdges;
    }


    //https://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html
    //check to see whether point is inside polygon
    //nvert is number of sides to the polygon, vertx and verty are the x and y coordinates of the polygon
    //testx and testy are the x and y coordinates of the point you want to check. It returns true if true false otherwise
    private boolean pnpoly(int nvert, double[] vertx, double[] verty, double testx, double testy)
    {
        int i, j = 0;
        boolean c = false;

        for (i = 0, j = nvert-1; i < nvert; j = i++)
        {
            if ( ((verty[i]>testy) != (verty[j]>testy)) && (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
            {
                c = !c;
            }
        }
        return c;
    }

}
