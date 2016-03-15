package gdx.tue.objects;

import com.badlogic.gdx.math.Vector2;

/**
 * Created by s138362 on 15-3-2016.
 */
public class Node
{

    private String name;
    private String label = "";
    private String cluster = "";
    private int index;

    private Vector2 pos;

    public Node(String name, int index)
    {
        pos = new Vector2(0, 0);
        this.name = name;
        this.index = index;
    }

    public void addCluster( String cluster )
    {
        this.cluster = cluster;
    }

    public void addLabel(String label)
    {
        this.label = label;
    }

    public void addPos( Vector2 pos )
    {
        this.pos = pos;
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

    public float getPosX() { return pos.x; }

    public float getPosY() { return pos.y; }
}