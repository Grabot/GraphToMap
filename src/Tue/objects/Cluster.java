package Tue.objects;

import Tue.load.Forces.CoulombForce;
import Tue.load.Forces.Force;
import Tue.load.Forces.FrictionForce;
import Tue.load.Forces.WallForce;
import Tue.load.voronoitreemap.j2d.Site;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;

/**
 * Created by s138362 on 31-3-2016.
 */
public class Cluster extends ClusterNode
{

    private ArrayList<Node> clusternodes = new ArrayList<Node>();
    private ArrayList<Cluster> neighbours = new ArrayList<Cluster>();
    private int clusternumber;

    private Site site = null;

    private double weight = 0;
    private double percentage = 100;

    public Cluster(int clusternumber, Force forces )
    {
        super( forces );
        this.clusternumber = clusternumber;
    }

    public void addNode( Node node )
    {
        clusternodes.add(node);
    }

    public ArrayList<Node> getNodes()
    {
        return clusternodes;
    }

    public void setPercentage( double percentage )
    {
        this.percentage = percentage;
        super.setFinalWeight(weight);
    }

    public double getPercentage()
    {
        return percentage;
    }

    public int getNumber()
    {
        return clusternumber;
    }

    public void addWeight( double weight )
    {
        this.weight = (this.weight + weight);
    }

    public double getWeight()
    {
        return weight;
    }

    public void setSite( Site site )
    {
        this.site = site;
    }

    public Site getSite()
    {
        return site;
    }

    public void setNeighbours( ArrayList<Cluster> neighbours )
    {
        this.neighbours = neighbours;
    }

    public ArrayList<Cluster> getNeighbours()
    {
        return neighbours;
    }

    public void draw(Graphics2D g2, double radius, Color color )
    {
        Color c1 = new Color(255, 255, 0, 200 );
        g2.setColor(c1);
        Ellipse2D.Double shape = new Ellipse2D.Double(this.getPos().x-(radius/2), this.getPos().y-(radius/2), radius, radius);
        g2.fill(shape);

        g2.setColor(Color.BLACK);
        Ellipse2D.Double shape2 = new Ellipse2D.Double(this.getSite().getPolygon().getCentroid().getX()-(radius/2), this.getSite().getPolygon().getCentroid().getY()-(radius/2), radius, radius);
        g2.fill(shape2);
    }

    public void drawText( Graphics2D g2 )
    {
        Font defaultFont = new Font("Arial", Font.BOLD, 20);
        g2.setFont(defaultFont);

        g2.drawString("cluster " + clusternumber, (float)this.getSite().getPolygon().getCentroid().getX(), (float)this.getSite().getPolygon().getCentroid().getY()-30 );
        g2.drawString("size " + (int)this.weight, (float)this.getSite().getPolygon().getCentroid().getX(), (float)this.getSite().getPolygon().getCentroid().getY()-10 );
    }
}
