package Tue.load.Display;

import Tue.load.Geometry.VoronoiCore;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;
import Tue.objects.DelaunayEdge;

import java.util.ArrayList;

/**
 * Created by s138362 on 13-4-2016.
 */
public class ForceDirectedMovement
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<DelaunayEdge> d_edges = new ArrayList<DelaunayEdge>();

    private float delta;
    private VoronoiCore core;

    private PolygonSimple boundingPolygon;

    private double moveXMax = 100;
    private double moveYMax = 50;

    public ForceDirectedMovement(PolygonSimple boundingPolygon, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, ArrayList<DelaunayEdge> d_edges, VoronoiCore core)
    {
        this.clusters = clusters;
        this.clusteredges = clusteredges;
        this.d_edges = d_edges;
        this.core = core;
        this.boundingPolygon = boundingPolygon;
    }

    public void ForceMove( float delta )
    {
        this.delta = delta;

        if( moveXMax != 0|| moveYMax != 0 )
        {
            calculatePosEuler();
        }
        //calculatePosMidPoint();
    }

    private void calculatePosMidPoint()
    {
        Vector2[] oldpos = new Vector2[clusters.size()];

        //we will use a midpoint calculation to numerically solve the differential equation

        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusters.size(); i++ )
        {
            oldpos[i] = clusters.get(i).getPos();
            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + (clusters.get(i).getVel().x * (delta/2))), (clusters.get(i).getPos().y + (clusters.get(i).getVel().y * (delta/2) ))));
        }
        calculateForces();
        for( int i = 0; i < clusters.size(); i++ )
        {
            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
            clusters.get(i).setPos( new Vector2((oldpos[i].x + (clusters.get(i).getVel().x * delta)), (oldpos[i].y + (clusters.get(i).getVel().y * delta ))));
        }
    }

    private void calculatePosEuler()
    {
        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusters.size(); i++ )
        {
            double xMove = (clusters.get(i).getForce().x * delta);
            double yMove = (clusters.get(i).getForce().y * delta);

            if( xMove > moveXMax )
            {
                xMove = moveXMax;
            }
            if( yMove > moveYMax )
            {
                yMove = moveYMax;
            }
            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + xMove), clusters.get(i).getPos().y + yMove));
        }

        moveXMax = (moveXMax-0.1);
        moveYMax = (moveYMax-0.05);

        if( moveXMax < 0 )
        {
            moveXMax = 0;
        }
        if( moveYMax < 0 )
        {
            moveYMax = 0;
        }

        core.moveSitesBack(clusters);
    }

    private void calculateForces()
    {
        //clear forces
        for (Cluster node : clusters) {
            node.setForce(new Vector2(0, 0));
        }

        getEdgeForces();
        getVoronoiForce();
        getNodeForces();
    }

    private void getVoronoiForce()
    {
        for( Cluster c : clusters )
        {
            double ks = 10;
            Site s = c.getSite();

            double distance = c.getPos().distance(new Vector2(s.getPolygon().getCentroid().getX(), s.getPolygon().getCentroid().getY()));
            double distanceX = c.getPos().getX() - s.getPolygon().getCentroid().getX();
            double distanceY = c.getPos().getY() - s.getPolygon().getCentroid().getY();

            double forceX = (-((distanceX/distance)*((ks * distance))));
            double forceY = (-((distanceY/distance)*((ks * distance))));

            c.setForce( new Vector2( c.getForce().getX() + (forceX), c.getForce().getY() + (forceY)));
        }
    }

    private void getEdgeForces()
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

    private void getNodeForces()
    {
        //apply forces node specific, so coulomb forces and wallforces
        for( Cluster node : clusters )
        {
            node.ApplyForces( clusters, delta );
        }
    }
}
