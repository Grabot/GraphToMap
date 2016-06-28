package Tue.load.Display;

import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.Cluster;
import Tue.objects.Node;

import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 22-6-2016.
 */
public class BeaconPositioning
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private Vector2 nodePosition = new Vector2(0, 0);

    private Random rand;
    private double step = 100;
    private double graphScaling;
    private double divideScale = 50;

    public BeaconPositioning(ArrayList<Cluster> clusters, ArrayList<Node> nodes, double graphScaling )
    {
        this.clusters = clusters;
        this.nodes = nodes;
        this.graphScaling = graphScaling;

        rand = new Random();
    }

    public void finalNodePositioning()
    {
        for( Cluster c : clusters )
        {
            if( c.getNodes().size() != 1 ) {
                for (Node n : c.getNodes()) {
                    PolygonSimple p = n.getSite().getPolygon();
                    n.setPos(new Vector2(p.getCentroid().getX(), p.getCentroid().getY()));
                }
            }
        }
    }

    public void positionClusterNodesFinal()
    {
        for( Cluster c : clusters )
        {
            PolygonSimple p = c.getSite().getPolygon();
            c.setPos( new Vector2(p.getCentroid().getX(), p.getCentroid().getY()));
        }
    }

    public void beacondBasedPositioning( double[][] clusterD, double[][] pairD, int width, int height )
    {
        for( Cluster cl : clusters )
        {
            PolygonSimple poly = cl.getSite().getPolygon();
            for (Node n : cl.getNodes()) {
                double[] nodeToCluster = new double[clusterD.length];

                for (Cluster c : clusters) {
                    double total = 0;
                    int amount = 0;
                    for (Node n2 : c.getNodes()) {
                        amount++;
                        total = (total + pairD[n.getIndex()][n2.getIndex()]);
                    }
                    if ((total / amount) == 0) {
                        nodeToCluster[c.getNumber()] = 0.01;
                    } else {
                        nodeToCluster[c.getNumber()] = (total / amount);
                    }
                    total = 0;
                    amount = 0;
                }
                //set the start position for the node

                double dist = 0;
                for( int i = 0; i < nodeToCluster.length; i++ )
                {
                    dist += nodeToCluster[i];
                }
                dist = (dist/nodeToCluster.length);

                double xPos = rand.nextDouble()*width;
                double yPos = rand.nextDouble()*height;
                while( !pnpoly(poly.length, poly.getXPoints(), poly.getYPoints(), xPos, yPos ))
                {
                    xPos = rand.nextDouble()*width;
                    yPos = rand.nextDouble()*height;
                }
                n.setPos(new Vector2( xPos, yPos ));
                while( step > 0.0001 )
                {
                    placeNode( n, nodeToCluster, dist );
                }
                step = 100;
            }
            scaleToCluster(cl);
        }
    }

    private void placeNode( Node n, double[] nodeToCluster, double dist )
    {
        double distortion = getDistortionNodeScale( n.getPos().getX(), n.getPos().getY(), nodeToCluster );
        Vector2 newPos = setNodePos( distortion, n, nodeToCluster, dist );
        if(! (newPos == null) )
        {
            step = (step*2);
            n.setPos(newPos);
        }
        else
        {
            step = (step/2);
        }
    }

    private double getDistortionNodeScale( double xPos, double yPos, double[] nodeToCluster )
    {
        double distortion = 0;

        double contractionlocal = 0;
        double expansionlocal = 0;
        int total = 0;

        double[] mapping = new double[clusters.size()];

        for (int i = 0; i < clusters.size(); i++) {
            Vector2 node1 = new Vector2(xPos, yPos);
            for (int j = 0; j < clusters.size(); j++)
            {
                Vector2 node2 = clusters.get(j).getPos();
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[j] = node1.distance(node2);
            }
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            total++;
            contractionlocal = (((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]) / mapping[i]);
            expansionlocal = (mapping[i] / ((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]));

            if( contractionlocal >= expansionlocal )
            {
                distortion += contractionlocal;
            }
            else if( expansionlocal >= contractionlocal )
            {
                distortion += expansionlocal;
            }
        }
        distortion = (distortion/total);

        return distortion;
    }

    private Vector2 setNodePos( double distortionInit, Node n, double[] nodeToCluster, double dist)
    {
        Vector2 gradient = getDerivativeExpansion( n.getPos().getX(), n.getPos().getY(), dist );
        Vector2 newPos = new Vector2(n.getPos().getX()-gradient.getX() * step, n.getPos().getY()-gradient.getY() * step);
        if( distortionInit < getDistortionNodeScale(newPos.getX(), newPos.getY(), nodeToCluster))
        {
            return null;
        }
        else
        {
            return newPos;
        }
    }

    private Vector2 getDerivativeExpansion( double xPos, double yPos, double dist)
    {
        double derivativeExpansionXTotal = 0;
        double derivativeExpansionYTotal = 0;

        for( Cluster c : clusters )
        {
            derivativeExpansionXTotal += getDerivativeExpansionX(xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
            derivativeExpansionYTotal += getDerivativeExpansionY(xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
        }

        derivativeExpansionXTotal = ( derivativeExpansionXTotal/clusters.size() );
        derivativeExpansionYTotal = ( derivativeExpansionYTotal/clusters.size() );

        return (new Vector2( derivativeExpansionXTotal, derivativeExpansionYTotal ));
    }

    private double getDerivativeExpansionX( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivate of expansion over x
        double result = (nodeX-clusterX)/(Math.sqrt(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2))*dist);

        return result;
    }

    private double getDerivativeExpansionY( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of expansion over y
        double result = (nodeY-clusterY)/(Math.sqrt(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2))*dist);

        return result;
    }

    public void finalPositioningCheck()
    {
        //it checks wheter 2 points are placed on top of each other, this messes up the voronoi creation.
        for( Node n1 : nodes )
        {
            for( Node n2 : nodes )
            {
                if( n1.getIndex() != n2.getIndex() )
                {
                    if( n1.getPos().equals(n2.getPos()))
                    {
                        //if this is the case, just move one a bit to the side.
                        n1.setPos(new Vector2((n1.getPos().getX()-0.1), (n1.getPos().getY()-0.1)));
                    }
                }
            }
        }
        singleClusterCheck();
    }

    private void singleClusterCheck()
    {
        //also if there is a cluster with only 1 node we will place that node in the cluster center
        for( Cluster c : clusters )
        {
            if( c.getNodes().size() == 1 )
            {
                Node n = c.getNodes().get(0);
                PolygonSimple p = c.getSite().getPolygon();
                n.setPos(new Vector2(p.getCentroid().getX(), p.getCentroid().getY() ));
                //we will also set the polygon for this node, since it won't have a site.
                n.setSite( c.getSite() );
            }
        }
    }

    public void scaleToCluster( Cluster cl )
    {
        //check if all the points are inside the cluster
        boolean inside = false;
        PolygonSimple p = cl.getSite().getPolygon();
        while( !inside ) {
            //we assume that all the nodes are inside, if that is the case we leave the loop
            inside = true;
            //check all the nodes of the cluster if they are inside the cluster.
            for (Node n : cl.getNodes()) {
                if (!pnpoly(p.getNumPoints(), p.getXPoints(), p.getYPoints(), n.getPos().getX(), n.getPos().getY())) {
                    //first mark it that it's not inside
                    inside = false;
                    //if a point is not inside, shrink the points till they do, this can be done for each node
                    Vector2 center = new Vector2(p.getCentroid().getX(), p.getCentroid().getY());
                    for (Node nMove : cl.getNodes()) {
                        //first translate it to the center, so the center of the cluster should be (0, 0)
                        n.setPos(new Vector2(n.getPos().getX() - center.getX(), n.getPos().getY() - center.getY()));
                        //scale it down
                        n.setPos(new Vector2(n.getPos().getX() * 0.999, n.getPos().getY() * 0.999));
                        //translate it back
                        n.setPos(new Vector2(n.getPos().getX() + center.getX(), n.getPos().getY() + center.getY()));
                    }
                }
            }
        }
    }


    public void positionNodesRandom( int width, int height)
    {
        double xPos = 0;
        double yPos = 0;

        PolygonSimple poly = null;

        for( Cluster c : clusters )
        {
            poly = c.getSite().getPolygon();
            for( Node n : c.getNodes() )
            {
                xPos = rand.nextDouble()*width;
                yPos = rand.nextDouble()*height;

                while( !pnpoly(poly.length, poly.getXPoints(), poly.getYPoints(), xPos, yPos ))
                {
                    xPos = rand.nextDouble()*width;
                    yPos = rand.nextDouble()*height;
                }
                n.setPos(new Vector2( xPos, yPos ));
            }
        }
    }

    public void RandomizeClusterNodes( Cluster cl )
    {
        double xPos = 0;
        double yPos = 0;

        PolygonSimple poly = null;

        poly = cl.getSite().getPolygon();
        for( Node n : cl.getNodes() )
        {
            xPos = rand.nextDouble()*1200;
            yPos = rand.nextDouble()*800;

            while( !pnpoly(poly.length, poly.getXPoints(), poly.getYPoints(), xPos, yPos ))
            {
                xPos = rand.nextDouble()*1200;
                yPos = rand.nextDouble()*800;
            }
            n.setPos(new Vector2( xPos, yPos ));
        }
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
