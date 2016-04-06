package Tue.objects;

import java.awt.*;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 6-4-2016.
 */
public class DelaunayEdge
{

    private Cluster source;
    private Cluster dest;

    public DelaunayEdge( Cluster source, Cluster dest )
    {
        this.source = source;
        this.dest = dest;
    }

    public Cluster getSource()
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
        Shape shape = new Line2D.Double(this.getSource().getPos().x, this.getSource().getPos().y, this.getDest().getPos().x, this.getDest().getPos().y);
        g2.draw(shape);
    }

}
