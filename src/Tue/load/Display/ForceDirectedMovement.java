package Tue.load.Display;

import Tue.load.Geometry.VoronoiCore;
import Tue.load.Vector2;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;

import java.util.ArrayList;

/**
 * Created by s138362 on 13-4-2016.
 */
public class ForceDirectedMovement
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();

    private float delta;
    private VoronoiCore core;

    public ForceDirectedMovement(ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, VoronoiCore core)
    {
        this.clusters = clusters;
        this.clusteredges = clusteredges;
        this.core = core;
    }

    public void ForceMove( float delta )
    {
        this.delta = delta;

        calculatePosEuler();
        core.voroDiagram();
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
            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + (clusters.get(i).getVel().x * delta)), (clusters.get(i).getPos().y + (clusters.get(i).getVel().y * delta ))));
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
        getNodeForces();
    }

    private void getEdgeForces()
    {
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate

        for (ClusterEdge edge : clusteredges) {
            edge.ApplyForces();
        }

//        for( DelaunayEdge edge : d_edges )
//        {
//            edge.ApplyForces();
//        }
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
