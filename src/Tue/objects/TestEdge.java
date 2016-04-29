package Tue.objects;

import Tue.load.Forces.Force;
import Tue.load.Vector2;

import java.awt.*;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 29-4-2016.
 */
public class TestEdge
{

    private Node source;
    private Cluster dest;
    private Force forces;

    private double weight = 0;

    public TestEdge( Node source, Cluster dest, Force forces )
    {
        this.source = source;
        this.dest = dest;
        this.forces = forces;
    }

    public Node getSource()
    {
        return source;
    }

    public Cluster getDest()
    {
        return dest;
    }

    public void draw(Graphics2D g2, Color color )
    {
        g2.setColor(color);
        Shape shape = new Line2D.Double(this.getSource().getX(), this.getSource().getY(), this.getDest().getPos().x, this.getDest().getPos().y);
        g2.draw(shape);
    }

    public void ApplyForces()
    {
        forces.ApplyEdgeForce( this );
    }

    public double getWeight()
    {
        return weight;
    }

    public void setWeight( double weight )
    {
        this.weight = weight;
    }

}
