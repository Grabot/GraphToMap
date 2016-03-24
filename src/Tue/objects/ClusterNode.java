package Tue.objects;

import Tue.load.Vector2;

import java.util.ArrayList;

/**
 * Created by s138362 on 24-3-2016.
 */
public class ClusterNode
{

    ArrayList<ClusterEdge> connections = new ArrayList<ClusterEdge>();
    private int cluster = -1;

    private Vector2 pos;

    public ClusterNode( int cluster )
    {
        this.cluster = cluster;
    }

    public void setPos( Vector2 pos )
    {
        this.pos = pos;
    }

    public Vector2 getPos()
    {
        return pos;
    }

}
