package gdx.tue.objects;

/**
 * Created by s138362 on 15-3-2016.
 */
public class Edge
{

    Node origin;
    Node dest;

    public Edge( Node origin, Node dest )
    {
        this.origin = origin;
        this.dest = dest;
    }

    public Node getFrom()
    {
        return origin;
    }

    public Node getDest()
    {
        return dest;
    }
}