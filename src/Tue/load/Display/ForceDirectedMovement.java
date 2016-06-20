package Tue.load.Display;

import Tue.load.Geometry.VoronoiCore;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.util.ArrayList;

/**
 * Created by s138362 on 13-4-2016.
 */
public class ForceDirectedMovement
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<DelaunayEdge> d_edges = new ArrayList<DelaunayEdge>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    private float delta;

    private PolygonSimple boundingPolygon;
    private PolygonSimple boundingClusters[];

    private double moveXMax = 100;
    private double moveYMax = 50;

    private boolean movementDone = false;

    public ForceDirectedMovement(PolygonSimple boundingPolygon, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, ArrayList<DelaunayEdge> d_edges)
    {
        this.clusters = clusters;
        this.clusteredges = clusteredges;
        this.d_edges = d_edges;
        this.boundingPolygon = boundingPolygon;
        boundingClusters = new PolygonSimple[clusters.size()];
    }

    public ForceDirectedMovement(ArrayList<Node> nodes, ArrayList<Edge> edges)
    {
        this.nodes = nodes;
        this.edges = edges;
    }

    public void ForceMoveNormal( float delta ) {
        this.delta = delta;
        if (moveXMax != 0 || moveYMax != 0) {
            calculatePosEulerNormal();
        }
        else
        {
            movementDone = true;
        }
    }

    public void ForceMoveCluster( float delta )
    {
        this.delta = delta;

        calculatePosEulerCluster();
        //calculatePosMidPoint();
    }

    private void calculatePosEulerNormal()
    {
        calculateForcesNormal();

        for( int i = 0; i < nodes.size(); i++ )
        {
            double xMove = (nodes.get(i).getForce().x * delta);
            double yMove = (nodes.get(i).getForce().y * delta);

            Vector2 newPos = new Vector2((nodes.get(i).getPos().x + xMove), nodes.get(i).getPos().y + yMove);

            PolygonSimple poly = nodes.get(i).getSite().getPolygon();
            if( poly == null )
            {
                xMove = 0;
                yMove = 0;
                newPos = new Vector2((nodes.get(i).getPos().x + xMove), nodes.get(i).getPos().y + yMove);
                nodes.get(i).setPos( newPos );
            }
            else if( pnpoly( poly.length, poly.getXPoints(), poly.getYPoints(), newPos.getX(), newPos.getY() ))
            {
                nodes.get(i).setPos( newPos );
            }
            else
            {
                xMove = (xMove/5);
                yMove = (yMove/5);
                newPos = new Vector2((nodes.get(i).getPos().x + xMove), nodes.get(i).getPos().y + yMove);
                nodes.get(i).setPos( newPos );
            }
        }

        moveXMax = (moveXMax-0.01);
        moveYMax = (moveYMax-0.005);

        if( moveXMax < 0 )
        {
            moveXMax = 0;
        }
        if( moveYMax < 0 )
        {
            moveYMax = 0;
        }
    }

    public boolean getMovement()
    {
        return movementDone;
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

    private void calculatePosEulerCluster()
    {
        calculateForcesCluster();

        double boundingArea = boundingPolygon.getArea();
        double siteError = 0;

        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusters.size(); i++ )
        {
            double xMove = (clusters.get(i).getForce().x * delta);
            double yMove = (clusters.get(i).getForce().y * delta);

            double sitePercentage = clusters.get(i).getSite().getPercentage();
            double siteArea = boundingArea*sitePercentage;

            PolygonSimple poly = clusters.get(i).getSite().getPolygon();
            if( poly == null )
            {
                xMove = 0;
                yMove = 0;
            }
            else {
                siteError = (poly.getArea() / siteArea);
                siteError = ((siteError - 1) * 2);
                if (siteError < 0) {
                    siteError = (siteError * -1);
                }
                if (siteError > 1) {
                    //1 would be the complete movement it wants to do
                    siteError = 1;
                }
                xMove = (xMove*siteError);
                yMove = (yMove*siteError);
            }


            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + xMove), clusters.get(i).getPos().y + yMove));
        }
    }

    private void calculateForcesNormal()
    {
        for( Node n : nodes )
        {
            n.setForce( new Vector2(0, 0));
        }

        getEdgeForcesNormal();
        getVoronoiForceNormal();
    }

    private void calculateForcesCluster()
    {
        //clear forces
        for (Cluster node : clusters) {
            node.setForce(new Vector2(0, 0));
        }

        getEdgeForcesCluster();
        getVoronoiForceCluster();
        getNodeForcesCluster();
    }

    private void getEdgeForcesNormal()
    {
        for( Edge e : edges )
        {
            e.ApplyForces();
        }
    }

    private void getVoronoiForceNormal()
    {
        for( Node n : nodes )
        {
            double ks = 3;
            Site s = n.getSite();

            PolygonSimple poly = s.getPolygon();
            double distance = 0;
            double distanceX = 0;
            double distanceY = 0;
            if( poly == null )
            {
                distance = 0;
                distanceX = 0;
                distanceY = 0;
            }
            else {
                distance = n.getPos().distance(new Vector2(poly.getCentroid().getX(), poly.getCentroid().getY()));
                distanceX = n.getPos().getX() - s.getPolygon().getCentroid().getX();
                distanceY = n.getPos().getY() - s.getPolygon().getCentroid().getY();
            }

            double forceX = 0;
            double forceY = 0;
            if( distance != 0 ) {
                forceX = (-((distanceX / distance) * ((ks * distance))));
                forceY = (-((distanceY / distance) * ((ks * distance))));
            }

            n.setForce( new Vector2( n.getForce().getX() + (forceX), n.getForce().getY() + (forceY)));
        }
    }

    private void getVoronoiForceCluster()
    {
        for( Cluster c : clusters )
        {
            double ks = 15;
            Site s = c.getSite();

            PolygonSimple poly = s.getPolygon();
            double distance = 0;
            double distanceX = 0;
            double distanceY = 0;
            if( poly == null )
            {
                distance = 0;
                distanceX = 0;
                distanceY = 0;
            }
            else {
                distance = c.getPos().distance(new Vector2(s.getPolygon().getCentroid().getX(), s.getPolygon().getCentroid().getY()));
                distanceX = c.getPos().getX() - s.getPolygon().getCentroid().getX();
                distanceY = c.getPos().getY() - s.getPolygon().getCentroid().getY();
            }

            double forceX = (-((distanceX/distance)*((ks * distance))));
            double forceY = (-((distanceY/distance)*((ks * distance))));

            c.setForce( new Vector2( c.getForce().getX() + (forceX), c.getForce().getY() + (forceY)));
        }
    }

    private void getEdgeForcesCluster()
    {
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate

//        for (ClusterEdge edge : clusteredges) {
//            edge.ApplyForces();
//        }

        for( DelaunayEdge edge : d_edges )
        {
            edge.ApplyForces();
        }
    }

    private void getNodeForcesCluster()
    {
        //apply forces node specific, so coulomb forces and wallforces
        for( Cluster node : clusters )
        {
            node.ApplyForces( clusters, delta );
        }
    }



//    private void calculatePosMidPoint()
//    {
//        Vector2[] oldpos = new Vector2[clusters.size()];
//
//        //we will use a midpoint calculation to numerically solve the differential equation
//
//        calculateForcesCluster();
//        //calculate velocity and position for all nodes.
//        for( int i = 0; i < clusters.size(); i++ )
//        {
//            oldpos[i] = clusters.get(i).getPos();
//            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
//            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + (clusters.get(i).getVel().x * (delta/2))), (clusters.get(i).getPos().y + (clusters.get(i).getVel().y * (delta/2) ))));
//        }
//        calculateForcesCluster();
//        for( int i = 0; i < clusters.size(); i++ )
//        {
//            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
//            clusters.get(i).setPos( new Vector2((oldpos[i].x + (clusters.get(i).getVel().x * delta)), (oldpos[i].y + (clusters.get(i).getVel().y * delta ))));
//        }
//    }

}
