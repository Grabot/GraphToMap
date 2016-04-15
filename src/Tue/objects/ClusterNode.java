package Tue.objects;

import Tue.load.Forces.Force;
import Tue.load.Vector2;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;

/**
 * Created by s138362 on 24-3-2016.
 */
public class ClusterNode
{

    ArrayList<ClusterEdge> connections = new ArrayList<ClusterEdge>();

    private Vector2 pos;
    private Vector2 vel;
    private Vector2 force;

    private Force forces;

    private double weight = -1;

    public ClusterNode( Force forces )
    {
        pos = new Vector2(0, 0);
        vel = new Vector2(0, 0);
        force = new Vector2(0, 0);
        this.forces = forces;
    }

    public void setPos( Vector2 pos )
    {
        this.pos = pos;
    }

    public Vector2 getPos()
    {
        return pos;
    }

    public void setVel( Vector2 vel )
    {
        this.vel = vel;
    }

    public Vector2 getVel()
    {
        return vel;
    }

    public void setForce( Vector2 force )
    {
        this.force = force;
    }

    public Vector2 getForce()
    {
        return force;
    }

    public void ApplyForces( ArrayList<Cluster> clusters, float delta )
    {
        forces.ApplyNodeForces( this, clusters, delta );
    }

    public void setFinalWeight( double weight ){
        this.weight = weight;
    }

}
