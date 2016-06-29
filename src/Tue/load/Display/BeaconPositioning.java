package Tue.load.Display;

import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.Cluster;
import Tue.objects.Node;
import Tue.objects.PolygonEdge;

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

    private double lowest = 999;
    private double highest = 0;

    private double clusterBoundary = 100;

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

    public void beacondBasedPositioning( double[][] clusterD, double[][] pairD, int width, int height, Renderer render )
    {

        Cluster cl = clusters.get(0);
        Node n = cl.getNodes().get(0);
//        for( Cluster cl : clusters )
//        {
            PolygonSimple poly = cl.getSite().getPolygon();
            //for (Node n : cl.getNodes()) {
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
                placeNode( cl, n, nodeToCluster, dist );

                render.setBeaconBasedTest( nodeToCluster, cl, n );
//                while( step > 0.0001 )
//                {
//                    placeNode( cl, n, nodeToCluster, dist );
//                }
//                step = 100;
//            }
//            scaleToCluster(cl);
//        }
    }

    private void placeNode( Cluster cl, Node n, double[] nodeToCluster, double dist )
    {
        double distortion = getDistortionNodeScale( n.getPos().getX(), n.getPos().getY(), cl, nodeToCluster );
        Vector2 newPos = setNodePos( distortion, cl, n, nodeToCluster, dist );
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

    private double getDistortionNodeScale( double xPos, double yPos, Cluster cl, double[] nodeToCluster )
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

        distortion = addBoundaryFunction( xPos, yPos, cl, distortion );

        return distortion;
    }

    private double addBoundaryFunction( double xPos, double yPos, Cluster cl, double distortionInit )
    {
        double distortion = distortionInit;

        ArrayList<PolygonEdge> polygonEdges = new ArrayList<PolygonEdge>();
        PolygonSimple poly = cl.getSite().getPolygon();
        double[] xPoints = poly.getXPoints();
        double[] yPoints = poly.getYPoints();

        for( int i = 0; i < poly.getNumPoints(); i++ ) {
            Vector2 from;
            Vector2 to;
            if ((i == (poly.getNumPoints() - 1))) {
                from = new Vector2(xPoints[i], yPoints[i]);
                to = new Vector2(xPoints[0], yPoints[0]);
            } else {
                from = new Vector2(xPoints[i], yPoints[i]);
                to = new Vector2(xPoints[i + 1], yPoints[i + 1]);
            }
            polygonEdges.add(new PolygonEdge(from, to));
        }

        if( pnpoly(poly.getNumPoints(), xPoints, yPoints, xPos, yPos ))
        {
            double boundaryCluster = 0;
            for( PolygonEdge e : polygonEdges )
            {
                double boundaryEdge = boundaryFunctionEdge( xPos, yPos, e.getFrom().getX(), e.getFrom().getY(), e.getTo().getX(), e.getTo().getY(), 100 );
                if( boundaryCluster < boundaryEdge )
                {
                    boundaryCluster = boundaryEdge;
                }
            }
            System.out.println("boundaryCluster: " + boundaryCluster );
            if( boundaryCluster > 10 )
            {
                boundaryCluster = 10;
            }
            distortion = (distortion+boundaryCluster);
        }
        else
        {
            distortion = (distortion+10);
        }
        return distortion;
    }

    private double boundaryFunctionEdge(  double x1, double y1, double x2, double y2, double x3, double y3, double dist )
    {
        double numerator = dist*Math.sqrt(Math.pow((x3-x2),2) + Math.pow((y3-y2), 2));
        double denominator = ((x1*(y2-y3))+(x2*(y3-y1))+(x3*(y1-y2)));

        return (numerator/denominator);
    }

    private Vector2 setNodePos( double distortionInit, Cluster cl, Node n, double[] nodeToCluster, double dist)
    {
        double distToCluster = distanceToCluster( cl, n );

        System.out.println( "distToCluster: " + distToCluster );

        Vector2 gradientExpansion = getDerivativeExpansion( n.getPos().getX(), n.getPos().getY(), dist );
        Vector2 gradientContraction = getDerivativeContraction( n.getPos().getX(), n.getPos().getY(), dist );

        //find which function is greater.
        if( Math.abs(gradientContraction.getX()) > Math.abs(gradientExpansion.getX()) )
        {
            gradientExpansion.x = gradientContraction.getX();
        }

        if( Math.abs(gradientContraction.getY()) > Math.abs(gradientExpansion.getY()) )
        {
            gradientExpansion.y = gradientContraction.getY();
        }

        Vector2 newPos = new Vector2(n.getPos().getX()-gradientExpansion.getX() * step, n.getPos().getY()-gradientExpansion.getY() * step);
        if( distortionInit < getDistortionNodeScale(newPos.getX(), newPos.getY(), cl, nodeToCluster))
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

    private Vector2 getDerivativeContraction( double xPos, double yPos, double dist )
    {
        double derivativeContractionXTotal = 0;
        double derivativeContractionYTotal = 0;

        for( Cluster c : clusters )
        {
            derivativeContractionXTotal += getDerivativeContractionX( xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
            derivativeContractionYTotal += getDerivativeContractionY( xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
        }

        derivativeContractionXTotal = ( derivativeContractionXTotal/clusters.size() );
        derivativeContractionYTotal = ( derivativeContractionYTotal/clusters.size() );

        return (new Vector2( derivativeContractionXTotal, derivativeContractionYTotal ));
    }

    private double getDerivativeContractionX( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of contraction over x
        double result = (nodeX-clusterX)*dist/(Math.pow(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2), (3/2)));

        return result;
    }

    private double getDerivativeContractionY( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of contraction over y
        double result = (nodeY-clusterY)*dist/(Math.pow(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2), (3/2)));

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

    private void scaleToCluster( Cluster cl )
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

    private double distanceToCluster( Cluster cl, Node n )
    {
        double result = 0;
        Vector2 derivativeEdge = null;
        double closest = 9999;
        PolygonSimple poly = cl.getSite().getPolygon();
        double[] xPoints = poly.getXPoints();
        double[] yPoints = poly.getYPoints();

        for( int i = 0; i < poly.getNumPoints(); i++ )
        {
            Vector2 from;
            Vector2 to;
            if((i == (poly.getNumPoints()-1)))
            {
                from = new Vector2( xPoints[i], yPoints[i] );
                to = new Vector2( xPoints[0], yPoints[0] );
            }
            else
            {
                from = new Vector2( xPoints[i], yPoints[i] );
                to = new Vector2( xPoints[i+1], yPoints[i+1] );
            }
            PolygonEdge edge = new PolygonEdge( from, to );

            result = edge.distanceToEdge(n);
            if( result < closest )
            {
                closest = result;
            }

            if( result < clusterBoundary ) {
                derivativeEdge = getDerivativePolyEdge(n.getPos().getX(), n.getPos().getY(), from.getX(), from.getY(), to.getX(), to.getY(), clusterBoundary);
                System.out.println("derivative x: " + derivativeEdge.getX() + " y: " + derivativeEdge.getY() );
            }
        }

        result = clusterBoundary/closest;

        if( result > 1 )
        {
            return result;
        }
        else {
            return 1;
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



    private Vector2 getDerivativePolyEdge( double x1, double y1, double x2, double y2, double x3, double y3, double dist )
    {
        double x = getDerivativePolyEdgeX( x1, y1, x2, y2, x3, y3, dist );
        double y = getDerivativePolyEdgeY( x1, y1, x2, y2, x3, y3, dist );
        Vector2 edgeGradient = new Vector2(x, y);
        return edgeGradient;
    }

    private double getDerivativePolyEdgeX( double x1, double y1, double x2, double y2, double x3, double y3, double c )
    {
        double numerator = c*((y2-y3)*Math.sqrt(Math.pow((x3 - x2), 2) + Math.pow((y3 - y2), 2)));
        double denominator = Math.pow(((x3*(y1-y2))+(x1*(y2-y3))+(x2*(y3-y1))),2);

        return ((numerator/denominator)*-1);
    }

    private double getDerivativePolyEdgeY( double x1, double y1, double x2, double y2, double x3, double y3, double c )
    {
        double numerator = c*((x3-x2)*Math.sqrt(Math.pow((x3 - x2), 2) + Math.pow((y3 - y2), 2)));
        double denominator = Math.pow(((x3*(y1-y2))+(x1*(y2-y3))+(x2*(y3-y1))),2);

        return ((numerator/denominator)*-1);
    }

}
