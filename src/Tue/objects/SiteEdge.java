package Tue.objects;

import Tue.load.Forces.Force;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;

/**
 * Created by s138362 on 12-4-2016.
 */
public class SiteEdge
{

    private Force forces;

    private Cluster source;
    private Vector2 dest;

    private PolygonSimple bounding;

    private double weight = 0;

    public SiteEdge(Cluster source, Force forces, PolygonSimple bounding )
    {
        this.forces = forces;
        this.source = source;
        this.bounding = bounding;
    }

    public Cluster getSource()
    {
        return source;
    }

    public Vector2 getDest()
    {
        dest = new Vector2(source.getSite().getPolygon().getCentroid().getX(), source.getSite().getPolygon().getCentroid().getY());
        return dest;
    }

    public void addDest( Vector2 dest )
    {
        this.dest = dest;
    }

    public double getWeight()
    {
        return weight;
    }

    public void ApplyForces()
    {
        forces.ApplySiteEdgeForce( this );
    }

    public PolygonSimple getBounding()
    {
        return bounding;
    }

}
