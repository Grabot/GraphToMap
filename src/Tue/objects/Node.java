package Tue.objects;


import Tue.load.Vector2;

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
}