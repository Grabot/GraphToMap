package Tue.objects;

import Tue.load.Forces.CoulombForce;
import Tue.load.Forces.FrictionForce;
import Tue.load.Forces.WallForce;
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

    private WallForce wall;
    private FrictionForce friction;
    private CoulombForce coulomb;

    public ClusterNode( WallForce wall, FrictionForce friction, CoulombForce coulomb )
    {
        pos = new Vector2(0, 0);
        vel = new Vector2(0, 0);
        force = new Vector2(0, 0);
        this.wall = wall;
        this.friction = friction;
        this.coulomb = coulomb;
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

    public void ApplyForces( ArrayList<Cluster> clusternodes, float delta )
    {
        wall.ApplyForces( this, delta );
        friction.ApplyForces( this );
        //coulomb.ApplyForces( this, clusternodes );
    }

    public void draw( Graphics2D g2, double radius, Color color )
    {
        g2.setColor(color);
        Ellipse2D.Double shape = new Ellipse2D.Double(this.getPos().x-(radius/2), this.getPos().y-(radius/2), radius, radius);
        g2.fill(shape);
    }

}
