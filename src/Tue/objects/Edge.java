package Tue.objects;

import Tue.load.Forces.Force;

import java.awt.*;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Edge
{
    private double weight = 1;
    Node source;
    Node dest;
    Force forces;

    public Edge( Node source, Node dest, Force forces)
    {
        this.source = source;
        this.dest = dest;
        this.forces = forces;
    }

    public void setWeight( double weight )
    {
        this.weight = weight;
    }

    public Node getSource()
    {
        return source;
    }

    public Node getDest()
    {
        return dest;
    }

    public double getWeight()
    {
        return weight;
    }

    public void draw(Graphics2D g2, Color color )
    {
        if( dest != null && source != null ) {
            g2.setColor(color);
            Shape shape = new Line2D.Double(this.getSource().getPos().x, this.getSource().getPos().y, this.getDest().getPos().x, this.getDest().getPos().y);
            g2.draw(shape);
        }
    }

    public void ApplyForces()
    {
        forces.ApplyEdgeForce( this );
    }
}