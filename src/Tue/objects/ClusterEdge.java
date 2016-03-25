package Tue.objects;

import Tue.load.Forces.SpringForce;

/**
 * Created by s138362 on 24-3-2016.
 */
public class ClusterEdge
{

    private ClusterNode source;
    private ClusterNode dest;
    private float weight = 0;
    private SpringForce spring;

    public ClusterEdge( ClusterNode source, ClusterNode dest, float weight, SpringForce spring )
    {
        this.source = source;
        this.dest = dest;
        this.weight = weight;
        this.spring = spring;
    }

    public ClusterNode getSource()
    {
        return source;
    }

    public ClusterNode getDest()
    {
        return dest;
    }


    public void ApplyForces()
    {
        spring.ApplyForce( this );
    }

    public void setWeight( float weight )
    {
        this.weight = weight;
    }

    public float getWeight()
    {
        return weight;
    }
}
