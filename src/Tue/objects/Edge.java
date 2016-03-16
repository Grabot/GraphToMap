package Tue.objects;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Edge
{

    Node source;
    Node dest;

    public Edge( Node source, Node dest )
    {
        this.source = source;
        this.dest = dest;
    }

    public Node getSource()
    {
        return source;
    }

    public Node getDest()
    {
        return dest;
    }
}