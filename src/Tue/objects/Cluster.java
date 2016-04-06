package Tue.objects;

import Tue.load.Forces.CoulombForce;
import Tue.load.Forces.Force;
import Tue.load.Forces.FrictionForce;
import Tue.load.Forces.WallForce;
import Tue.load.voronoitreemap.j2d.Site;

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
        super( clusternumber, forces );
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
}
