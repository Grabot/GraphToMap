package Tue.load.Forces;

import Tue.objects.*;

import java.util.ArrayList;

/**
 * Created by s138362 on 6-4-2016.
 */
public class Force
{

    private SpringForce spring;
    private WallForce wall;
    private FrictionForce friction;
    private CoulombForce coulomb;

    public Force( int width, int height )
    {
        spring = new SpringForce();
        wall = new WallForce(width, height);
        friction = new FrictionForce();
        coulomb = new CoulombForce();
    }

    public void ApplyNodeForces( ClusterNode node, ArrayList<Cluster> clusters, float delta )
    {
        wall.ApplyForces(node, delta);
        friction.ApplyForces(node);
        //coulomb.ApplyForces(node, clusters);
    }

    public void ApplyEdgeForce( ClusterEdge edge )
    {
        spring.ApplyForce(edge);
    }

    public void ApplyEdgeForce( DelaunayEdge edge )
    {
        spring.ApplyForce( edge );
    }

    public void ApplyEdgeForce( Edge edge )
    {
        spring.ApplyForce( edge );
    }

}
