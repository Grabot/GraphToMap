package Tue.objects;

import Tue.load.Vector2;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;

/**
 * Created by s138362 on 27-6-2016.
 */
public class VectorGraphic
{

    private Vector2 pos;
    private Vector2 direction;
    private double radius = 4;
    private double magnify = 50;
    public VectorGraphic(Vector2 pos, Vector2 direction )
    {
        this.pos = pos;
        this.direction = direction;
    }

    public void draw(Graphics2D g2)
    {
        g2.setColor( new Color(0, 0, 0));
        //Line2D.Double line = new Line2D.Double(100, 100, 300, 300 );
        Line2D.Double line = new Line2D.Double(pos.getX(), pos.getY(), pos.getX()-(direction.getX()*magnify), pos.getY()-(direction.getY()*magnify) );
        Ellipse2D.Double circ = new Ellipse2D.Double((pos.getX()-(direction.getX()*magnify)-(radius/2)), (pos.getY()-(direction.getY()*magnify)-(radius/2)), radius, radius);
        g2.draw(line);
        g2.fill(circ);
    }

}
