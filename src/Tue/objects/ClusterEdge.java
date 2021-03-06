package Tue.objects;

import Tue.load.Forces.Force;
import Tue.load.Forces.SpringForce;

import java.awt.*;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 24-3-2016.
 */
public class ClusterEdge
{

    private Cluster source;
    private Cluster dest;
    private double weight = 0;
    private Force forces;

    public ClusterEdge( Cluster source, Cluster dest, double weight, Force forces )
    {
        this.source = source;
        this.dest = dest;
        this.weight = weight;
        this.forces = forces;
    }

    public ClusterNode getSource()
    {
        return source;
    }

    public ClusterNode getDest()
    {
        return dest;
    }


    public void ApplyForces()
    {
        forces.ApplyEdgeForce( this );
    }

    public void setWeight( float weight )
    {
        this.weight = weight;
    }

    public double getWeight()
    {
        return weight;
    }

    public void draw( Graphics2D g2, Color color )
    {
        g2.setColor(color);
        Shape shape = new Line2D.Double(this.getSource().getPos().x, this.getSource().getPos().y, this.getDest().getPos().x, this.getDest().getPos().y);
        g2.draw(shape);
    }
}
