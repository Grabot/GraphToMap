package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.diagram.PowerDiagram;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.BorderNode;
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

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();
    private ArrayList<BorderNode> waternodes = new ArrayList<BorderNode>();

    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;

    private double distanceBorder = 0;

    private int iterations = 0;

    public Simulation(Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height )
    {
        this.width = width;
        this.height = height;

        this.clusters = clusters;
        this.clusteredges = clusteredges;
        this.render = render;

        // normal list based on an array
        sites = new OpenList();
        core = new VoronoiCore();

        //create bounding polygon (the size of the panel)
        boundingPolygonTest();
        /*
        boundingPolygon = new PolygonSimple();
        boundingPolygon.add(0, 0);
        boundingPolygon.add(width, 0);
        boundingPolygon.add(width, height);
        boundingPolygon.add(0, height);
        render.addBounding(boundingPolygon);
        */

        int amount=clusters.size();

        for (int i=0;i<amount;i++){
            Site site = new Site(clusters.get(i).getPos().x, clusters.get(i).getPos().y);
            //site.setPercentage(rand.nextFloat());
            site.setPercentage(clusters.get(i).getPercentage()*100);
            sites.add(site);
        }

        core.normalizeSites(sites);

        core.setSites(sites);
        core.setClipPolygon(boundingPolygon);
        core.voroDiagram();
    }

    private void boundingPolygonTest()
    {
        //add random nodes around the cluster nodes as is stated in the GMap implementation
        while( true )
        {
            boolean place = true;
            double r = 130;
            Random random = new Random();
            double xPos = random.nextDouble()*width;
            double yPos = random.nextDouble()*height;

            for( Cluster node : clusters )
            {
                if(node.getPos().distance(new Vector2(xPos, yPos)) < r )
                {
                    place = false;
                }
            }

            if( place )
            {
                BorderNode border = new BorderNode(new Vector2(xPos, yPos));
                waternodes.add(border);
            }

            if( waternodes.size() == 1000 )
            {
                break;
            }
        }

        boundingPolygon = new PolygonSimple();
        double xPosmin = width;
        double xPosmax = 0;
        double yPosmin = height;
        double yPosmax = 0;

        for( Cluster node : clusters )
        {
            if( node.getPos().x < xPosmin )
            {
                xPosmin = node.getPos().x;
            }
            if( node.getPos().x > xPosmax )
            {
                xPosmax = node.getPos().x;
            }
            if( node.getPos().y < yPosmin )
            {
                yPosmin = node.getPos().y;
            }
            if( node.getPos().y > yPosmax )
            {
                yPosmax = node.getPos().y;
            }
        }

//        boundingPolygon.add(xPosmin - 100, yPosmin - 100 );
//        boundingPolygon.add(xPosmax + 100, yPosmin - 100 );
//        boundingPolygon.add(xPosmax + 100, yPosmax + 100 );
//        boundingPolygon.add(xPosmin - 100, yPosmax + 100 );
//        render.addBounding(boundingPolygon);

        boundingPolygon.add(0, 0 );
        boundingPolygon.add(width, 0 );
        boundingPolygon.add(width, height );
        boundingPolygon.add(0, height );
        render.addBounding(boundingPolygon);
    }

    public void update( float delta )
    {
        this.delta = delta;

        sites = core.getSites();

        //core.iterateSimple();
        //setClusterNodes();

//        calculatePos();
//        calculateForces();
//        setSiteNodes();
//        core.voroDiagram();

        render.addSites( core.getSites() );
        render.addBorderNodes( waternodes );
    }

    private void setSiteNodes()
    {
        //the sites will here only serve the voronoi diagram, so they can be randomly assigned
        //as long as each site is assigned to a cluster node.
        sites = core.getSiteList();
        int amount = clusters.size();
        for( int i = 0; i < amount; i++ )
        {
            sites.get(i).setXY( clusters.get(i).getPos().x, clusters.get(i).getPos().y );
        }
        core.setSiteList( sites );
    }

    private void setClusterNodes()
    {
        for( Site s : sites )
        {
            for( Cluster node : clusters )
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
        Vector2[] oldpos = new Vector2[clusters.size()];

        //we will use a midpoint calculation to numerically solve the differential equation

        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusters.size(); i++ )
        {
            oldpos[i] = clusters.get(i).getPos();
            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
            clusters.get(i).setPos( new Vector2((clusters.get(i).getPos().x + (clusters.get(i).getVel().x * (delta/2))), (clusters.get(i).getPos().y + (clusters.get(i).getVel().y * (delta/2) ))));
        }
        calculateForces();
        for( int i = 0; i < clusters.size(); i++ )
        {
            clusters.get(i).setVel( new Vector2((clusters.get(i).getVel().x + (clusters.get(i).getForce().x * delta)), (clusters.get(i).getVel().y + (clusters.get(i).getForce().y * delta ))));
            clusters.get(i).setPos( new Vector2((oldpos[i].x + (clusters.get(i).getVel().x * delta)), (oldpos[i].y + (clusters.get(i).getVel().y * delta ))));
        }
    }

    private void calculateForces()
    {
        for (Cluster node : clusters) {
            node.setForce(new Vector2(0, 0));
        }
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate
        for (ClusterEdge edge : clusteredges) {
            edge.ApplyForces();
        }

        for( Cluster node : clusters )
        {
            node.ApplyForces( clusters, delta );
        }
    }
}
