package Tue.objects;


import Tue.load.Vector2;

import java.awt.*;
import java.awt.geom.Ellipse2D;

/**
 * Created by s138362 on 15-3-2016.
 */
public class Node
{

    Vector2 pos;

    private String name;
    private String label = "";
    private String cluster = "";
    private int clusterNumber = -1;
    private int index;

    private Color color;

    private double weight = 1;

    public Node(String name, int index)
    {
        pos = new Vector2(0, 0);
        this.name = name;
        this.index = index;
    }

    public void setPos( Vector2 pos )
    {
        this.pos = pos;
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
}