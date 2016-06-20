package Tue.objects;


import Tue.load.Forces.Force;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
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

    private int textHeight = 0;

    private PolygonSimple rect;

    private int initialTextSize = 20;

    public Node(Force forces, String name, int index)
    {
        pos = new Vector2(0, 0);
        vel = new Vector2(0, 0);
        force = new Vector2(0, 0);
        this.forces = forces;
        this.name = name;
        this.index = index;

        rect = new PolygonSimple();
    }

    public PolygonSimple setRect(Graphics2D g2, double zoominverse)
    {
        Font defaultFont = new Font("Arial", Font.BOLD, (int)(initialTextSize*zoominverse));
        g2.setFont(defaultFont);

        String nodeName = ("" + this.getName());
        textHeight = (int)(initialTextSize*zoominverse);
        int textWidth = g2.getFontMetrics().stringWidth(nodeName);
        PolygonSimple nodePolygon = this.getSite().getPolygon();
        float centroidX = 0;
        float centroidY = 0;
        if( nodePolygon == null )
        {
            centroidX = (float)this.getX();
            centroidY = (float)this.getY();
        }
        else {
            centroidX = (float) nodePolygon.getCentroid().getX();
            centroidY = (float) nodePolygon.getCentroid().getY();
        }

        PolygonSimple rect = new PolygonSimple();
        rect.add((centroidX-(textWidth/2)), (centroidY+(textHeight/2)));
        rect.add((centroidX-(textWidth/2)), (centroidY-(textHeight/2)));
        rect.add((centroidX+(textWidth/2)), (centroidY-(textHeight/2)));
        rect.add((centroidX+(textWidth/2)), (centroidY+(textHeight/2)));

        return rect;
    }

    public void drawText( Graphics2D g2 )
    {
        g2.setColor(Color.BLACK);
        String nodeName = ("" + this.getName());
        int textWidth = g2.getFontMetrics().stringWidth(nodeName);
        PolygonSimple nodePolygon = this.getSite().getPolygon();
        float centroidX = 0;
        float centroidY = 0;
        if( nodePolygon == null )
        {
            centroidX = (float)this.getX();
            centroidY = (float)this.getY();
        }
        else {
            centroidX = (float) nodePolygon.getCentroid().getX();
            centroidY = (float) nodePolygon.getCentroid().getY();
        }
        g2.drawString(nodeName, (centroidX-(textWidth/2)), (centroidY+(textHeight/2)) );
    }

    public void drawNode( Graphics2D g2 )
    {
        Color c1 = new Color(27, 0, 255, 200 );
        g2.setColor(c1);
        Ellipse2D.Double shape = new Ellipse2D.Double(pos.x-(10/2), pos.y-(10/2), 10, 10);
        g2.fill(shape);
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

    public void setSite( Site s )
    {
        this.s = s;
    }

    public Site getSite()
    {
        return s;
    }
}