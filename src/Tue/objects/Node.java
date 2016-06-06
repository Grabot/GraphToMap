package Tue.objects;


import Tue.load.Forces.Force;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.Site;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;

/**
 * Created by s138362 on 15-3-2016.
 */
public class Node
{

    private Vector2 pos;
    private Vector2 vel;
    private Vector2 force;

    private String name;
    private String label = "";
    private String cluster = "";
    private int clusterNumber = -1;
    private int index;
    private Site s;

    private Color color;
    private Force forces;

    private double weight = 1;

    public Node(Force forces, String name, int index)
    {
        pos = new Vector2(0, 0);
        vel = new Vector2(0, 0);
        force = new Vector2(0, 0);
        this.forces = forces;
        this.name = name;
        this.index = index;
    }

    public void setPos( Vector2 pos )
    {
        this.pos = pos;
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

    public void addCluster( String cluster )
    {
        this.cluster = cluster;
    }

    public void addLabel(String label)
    {
        this.label = label;
    }

    public int getIndex() { return index; }

    public String getName()
    {
        return name;
    }

    public String getLabel()
    {
        return label;
    }

    public String getCluster() { return cluster; }

    public void addClusterNumber( int clusterNumber )
    {
        this.clusterNumber = clusterNumber;
    }

    public int getClusterNumber()
    {
        return clusterNumber;
    }

    public double getX() { return pos.x; }

    public double getY() { return pos.y; }

    public Vector2 getPos() { return pos; }

    public void addWeight(double weight)
    {
        this.weight = weight;
    }

    public double getWeight()
    {
        return weight;
    }

    public void setColor( Color color )
    {
        this.color = color;
    }

    public Color getColor()
    {
        return color;
    }

    public void draw(Graphics2D g2, double radius, Color color )
    {
        g2.setColor(color);
        Ellipse2D.Double shape2 = new Ellipse2D.Double(this.getX()-(radius/2), this.getY()-(radius/2), radius, radius);
        g2.fill(shape2);
    }

    public void draw2( Graphics2D g2, Color color )
    {
        Font defaultFont = new Font("Arial", Font.BOLD, 20);
        g2.setFont(defaultFont);

        g2.setColor(color);
        String nodeName = ("" + this.getName());
        int textWidth = g2.getFontMetrics().stringWidth(nodeName);
        int textHeight = 20;
        g2.drawString(nodeName, (float)(this.getSite().getPolygon().getCentroid().getX()-(textWidth/2)), (float)(this.getSite().getPolygon().getCentroid().getY()+(textHeight/2)) );
        Rectangle2D.Double shapeRect = new Rectangle2D.Double((float)(this.getSite().getPolygon().getCentroid().getX()-(textWidth/2)), (float)((this.getSite().getPolygon().getCentroid().getY()+(textHeight/2))-textHeight), textWidth, textHeight);
        g2.draw(shapeRect);
    }

    public void setSite( Site s )
    {
        this.s = s;
    }

    public Site getSite()
    {
        return s;
    }
}