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

    public BeaconPositioning(ArrayList<Cluster> clusters, ArrayList<Node> nodes)
    {
        this.clusters = clusters;
        this.nodes = nodes;

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


    public void beacondBasedPositioning( double[][] clusterD, double[][] pairD) {
        //this will find all intersecting points of any 2 circles and save them, later it will check if they engulf all points
        for( Cluster cl : clusters )
        {
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

                beaconIntersections(nodeToCluster);
                Vector2 foundPosition = new Vector2(nodePosition.x, nodePosition.y);
                nodes.get(n.getIndex()).setPos(foundPosition);
                System.out.println("found position node: " + n.getIndex() );
            }
            scaleToCluster(cl);
        }


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

    public void beaconIntersections( double[] nodeToCluster )
    {
        double[] distances = new double[nodeToCluster.length];
        double lambda = 0.1;

        for( int i = 0; i < 100000; i++ ) {
            for (int j = 0; j < nodeToCluster.length; j++) {
                distances[j] = nodeToCluster[j] * lambda;
            }
            lambda = (lambda + 0.1);
            if( circleIntersectionCheck(distances) )
            {
                break;
            }
        }
    }

    private boolean circleIntersectionCheck( double[] nodeToClusters )
    {
        ArrayList<Vector2> circleIntersections = new ArrayList<Vector2>();
        boolean intersects = true;

        double radiusR1 = 0;
        double radiusR2 = 0;
        double distance = 0;

        for( Cluster c1 : clusters ) {
            for( Cluster c2 : clusters )
            {
                if(c1.getNumber() != c2.getNumber())
                {
                    radiusR1 = nodeToClusters[c1.getNumber()];
                    radiusR2 = nodeToClusters[c2.getNumber()];
                    distance = c1.getPos().distance(c2.getPos());

                    if (distance > (radiusR1 + radiusR2))
                    {
                        //they are separate and don't intersect
                    }
                    else if( distance < (radiusR1 - radiusR2 ))
                    {
                        //one circle is contained within the other.
                    }
                    else
                    {
                        //find intersection points of the 2 overlapping circles.
                        //http://paulbourke.net/geometry/circlesphere/
                        //we can find the intersection based on the triangle between c1, newPos and the intersection.
                        //there should be 2 points, so 2 solutions for x, y
                        double dx = c2.getPos().getX()-c1.getPos().getX();
                        double dy = c2.getPos().getY()-c1.getPos().getY();

                        double a = (((radiusR1*radiusR1)-(radiusR2*radiusR2) + (distance*distance))/(2*distance));

                        double newX = c1.getPos().getX() + (dx*(a/distance));
                        double newY = c1.getPos().getY() + (dy*(a/distance));

                        Vector2 P2 = new Vector2( newX, newY );

                        //determine the distance from point P2 to the 2 intersection points.
                        double h = Math.sqrt(((radiusR1*radiusR1) - (a*a)));

                        double rx = (-dy * (h/distance));
                        double ry = (dx * (h/distance));

                        double x3 = P2.getX()+rx;
                        double y3 = P2.getY()+ry;

                        double x3Prime = P2.getX()-rx;
                        double y3Prime = P2.getY()-ry;

                        circleIntersections.add(new Vector2(x3, y3));
                        circleIntersections.add(new Vector2(x3Prime, y3Prime));
                    }
                }
            }
        }

        intersects = checkPointsIntersection( nodeToClusters, circleIntersections );

        return intersects;
    }

    private boolean checkPointsIntersection( double[] nodeToClusters, ArrayList<Vector2> circleIntersections )
    {
        if( circleIntersections.size() > 0 )
        {
            for( Vector2 point : circleIntersections )
            {
                if( !(Double.isNaN(point.getX()) && Double.isNaN(point.getY())) ) {
                    boolean intersects = true;
                    for (Cluster c : clusters) {
                        double radiusToPoint = point.distance(c.getPos());
                        double radiusCluster = nodeToClusters[c.getNumber()];

                        if (radiusCluster < radiusToPoint) {
                            intersects = false;
                        }
                    }
                    if (intersects) {
                        //there is a point which is in all the circle radii.
                        nodePosition = point;
                        return true;
                    }
                }
            }
        }
        else
        {
            return false;
        }
        return false;
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
