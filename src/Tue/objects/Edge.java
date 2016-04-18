package Tue.objects;

import java.awt.*;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Edge
{
    private int weight = 1;
    Node source;
    Node dest;

    public Edge( Node source, Node dest )
    {
        this.source = source;
        this.dest = dest;
    }

    public void setWeight()
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

    public int getWeight()
    {
        return weight;
    }

    public void draw(Graphics2D g2, Color color )
    {
        g2.setColor(color);
        Shape shape = new Line2D.Double(this.getSource().getPos().x, this.getSource().getPos().y, this.getDest().getPos().x, this.getDest().getPos().y);
        g2.draw(shape);
    }
}