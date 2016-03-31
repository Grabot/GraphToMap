package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.diagram.PowerDiagram;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;

import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Simulation
{

    private ArrayList<Cluster> clusternodes = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();

    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;

    private double distanceBorder = 0;

    private int iterations = 0;

    public Simulation(Renderer render, ArrayList<Cluster> clusternodes, ArrayList<ClusterEdge> clusteredges, int width, int height )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
        this.render = render;

        // normal list based on an array
        sites = new OpenList();
        core = new VoronoiCore();

        //create bounding polygon (the size of the panel)
        boundingPolygon = new PolygonSimple();
        boundingPolygon.add(0, 0);
        boundingPolygon.add(width, 0);
        boundingPolygon.add(width, height);
        boundingPolygon.add(0, height);
        render.addBounding(boundingPolygon);

        int amount=clusternodes.size();

        for (int i=0;i<amount;i++){
            Site site = new Site(clusternodes.get(i).getPos().x, clusternodes.get(i).getPos().y);
            //site.setPercentage(rand.nextFloat());
            site.setWeight(100);
            sites.add(site);
        }

        sites.get(0).setPercentage(2);
        sites.get(1).setPercentage(2);
        sites.get(2).setPercentage(2);
        sites.get(3).setPercentage(2);
        sites.get(4).setPercentage(2);
        sites.get(5).setPercentage(10);
        sites.get(6).setPercentage(40);
        sites.get(7).setPercentage(3);
        sites.get(8).setPercentage(3);
        sites.get(9).setPercentage(2);
        sites.get(10).setPercentage(2);

        core.normalizeSites(sites);

        core.setSites(sites);
        core.setClipPolygon(boundingPolygon);
        core.voroDiagram();
    }

    public void update( float delta )
    {
        this.delta = delta;

        sites = core.getSites();

        //core.iterateSimple();
        //setClusterNodes();

        calculatePos();
        calculateForces();
//        ComputePowerDiagram();

        //render.addSites( core.getSites() );
    }

    private void setClusterNodes()
    {
        for( Site s : sites )
        {
            for( ClusterNode node : clusternodes )
            {
                if((node.getPos().x == s.getOldX()) && (node.getPos().y == s.getOldY()))
                {
                    node.setPos(new Vector2(s.getX(), s.getY()));
                }
            }
        }
    }

    private void calculatePos()
    {
        Vector2[] oldpos = new Vector2[clusternodes.size()];

        //we will use a midpoint calculation to numerically solve the differential equation

        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            oldpos[i] = clusternodes.get(i).getPos();
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((clusternodes.get(i).getPos().x + (clusternodes.get(i).getVel().x * (delta/2))), (clusternodes.get(i).getPos().y + (clusternodes.get(i).getVel().y * (delta/2) ))));
        }
        calculateForces();
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((oldpos[i].x + (clusternodes.get(i).getVel().x * delta)), (oldpos[i].y + (clusternodes.get(i).getVel().y * delta ))));
        }
    }

    private void calculateForces()
    {
        for (ClusterNode node : clusternodes) {
            node.setForce(new Vector2(0, 0));
        }
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate
        for (ClusterEdge edge : clusteredges) {
            edge.ApplyForces();
        }

        for( ClusterNode node : clusternodes )
        {
            node.ApplyForces( clusternodes, delta );
        }
    }
}
