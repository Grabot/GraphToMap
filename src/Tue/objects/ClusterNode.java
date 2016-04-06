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

    private int clusternumber = -1;
    private double weight = -1;

    public ClusterNode( int clusternumber, Force forces )
    {
        pos = new Vector2(0, 0);
        vel = new Vector2(0, 0);
        force = new Vector2(0, 0);
        this.forces = forces;
        this.clusternumber = clusternumber;
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

    public void draw( Graphics2D g2, double radius, Color color )
    {
        g2.setColor(color);
        Ellipse2D.Double shape = new Ellipse2D.Double(this.getPos().x-(radius/2), this.getPos().y-(radius/2), radius, radius);
        g2.fill(shape);
        g2.drawString("cluster " + clusternumber, (float)this.getPos().x, (float)(this.getPos().y-30) );
        g2.drawString("size " + (int)this.weight, (float)this.getPos().x, (float)(this.getPos().y-20));
    }

}
