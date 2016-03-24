package Tue.objects;

/**
 * Created by s138362 on 24-3-2016.
 */
public class ClusterEdge
{

    private ClusterNode source;
    private ClusterNode dest;
    private float weight = 0;

    public ClusterEdge( ClusterNode source, ClusterNode dest, float weight )
    {
        this.source = source;
        this.dest = dest;
        this.weight = weight;
    }

    public ClusterNode getSource()
    {
        return source;
    }

    public ClusterNode getDest()
    {
        return dest;
    }
}
