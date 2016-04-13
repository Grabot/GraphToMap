package Tue.load.Display;

import Tue.load.Geometry.VoronoiCore;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;

import java.util.ArrayList;

/**
 * Created by s138362 on 13-4-2016.
 */
public class AreaMovement
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private VoronoiCore core;

    public AreaMovement( ArrayList<Cluster> clusters, VoronoiCore core )
    {
        this.clusters = clusters;
        this.core = core;
    }

    public void AreaMove()
    {
        core.adaptWeightsSimple();
        core.voroDiagram();
        positionClusterNode();
    }

    private void positionClusterNode()
    {
        for( Cluster c : clusters )
        {
            PolygonSimple p = c.getSite().getPolygon();
            c.setPos(new Vector2(p.getCentroid().x, p.getCentroid().y));
        }
    }
}
