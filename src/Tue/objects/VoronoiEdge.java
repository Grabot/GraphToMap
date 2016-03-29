package Tue.objects;

import Tue.load.Vector2;

/**
 * Created by s138362 on 29-3-2016.
 */
public class VoronoiEdge
{

    private Vector2 source;
    private Vector2 dest;

    public VoronoiEdge( Vector2 source, Vector2 dest )
    {
        this.source = source;
        this.dest = dest;
    }

    public Vector2 getSource()
    {
        return source;
    }

    public Vector2 getDest()
    {
        return dest;
    }
}
