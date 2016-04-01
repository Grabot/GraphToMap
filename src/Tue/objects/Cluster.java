package Tue.objects;

import Tue.load.Forces.CoulombForce;
import Tue.load.Forces.FrictionForce;
import Tue.load.Forces.WallForce;

import java.util.ArrayList;

/**
 * Created by s138362 on 31-3-2016.
 */
public class Cluster extends ClusterNode
{

    private ArrayList<Node> clusternodes = new ArrayList<Node>();
    private int clusternumber;

    private double weight = 0;
    private double percentage = 100;

    public Cluster(int clusternumber, WallForce wall, FrictionForce friction, CoulombForce coulomb )
    {
        super( clusternumber, wall, friction, coulomb );
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

}
