package Tue.load;

import Tue.load.Geometry.ConvexHull;
import Tue.load.Geometry.VoronoiCore;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.BorderNode;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;
import Tue.objects.DelaunayEdge;

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
    private ArrayList<DelaunayEdge> d_edges = new ArrayList<DelaunayEdge>();
    private ArrayList<Vector2> borderpoints = new ArrayList<Vector2>();
    private boolean[][] neighbours;
    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;

    private double distanceBorder = 0;

    private int iterations = 0;

    private ConvexHull boundary;

    public Simulation(Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height )
    {
        this.width = width;
        this.height = height;

        this.clusters = clusters;
        this.clusteredges = clusteredges;
        this.render = render;

        neighbours = new boolean[clusters.size()][clusters.size()];

        // normal list based on an array
        sites = new OpenList();
        core = new VoronoiCore();

        boundary = new ConvexHull();
        testwithborder();
        //boundingPolygonTest();
        //boundingPolygon();

        for (int i=0;i<clusters.size();i++){
            Site site = new Site(clusters.get(i).getPos().x, clusters.get(i).getPos().y);
            //site.setPercentage(rand.nextFloat());
            site.setPercentage(clusters.get(i).getPercentage()*100);
            site.setIndex(clusters.get(i).getNumber());
            sites.add(site);
            borderpoints.add( new Vector2( clusters.get(i).getPos().x, clusters.get(i).getPos().y ));
        }

        core.normalizeSites(sites);

        core.setSites(sites);
        core.setClipPolygon(boundingPolygon);
        core.voroDiagram();


    }

    private void boundingPolygon()
    {
        boundingPolygon = new PolygonSimple();

        boundingPolygon.add(10, 10 );
        boundingPolygon.add(1170, 10 );
        boundingPolygon.add(1170, 750 );
        boundingPolygon.add(10, 750 );
        render.addBounding(boundingPolygon);
    }

    private void boundingPolygonTest()
    {

        boundingPolygon = new PolygonSimple();
        double xPosmin = Double.MAX_VALUE;
        double xPosmax = Double.MIN_VALUE;
        double yPosmin = Double.MAX_VALUE;
        double yPosmax = Double.MIN_VALUE;

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

        boundingPolygon.add(xPosmin - 100, yPosmin - 100 );
        boundingPolygon.add(xPosmax + 100, yPosmin - 100 );
        boundingPolygon.add(xPosmax + 100, yPosmax + 100 );
        boundingPolygon.add(xPosmin - 100, yPosmax + 100 );
        render.addBounding(boundingPolygon);

    }

    private void testwithborder()
    {
        Vector2[] points = new Vector2[clusters.size()];
        boundingPolygon = new PolygonSimple();

        for( int i = 0; i < clusters.size(); i++ )
        {
            points[i] = clusters.get(i).getPos();
        }
        boundary.setPoints(points);
        Vector2[] results = boundary.getConvexHull();

        for( int i = 0; i < results.length; i++ )
        {
            boundingPolygon.add( results[i].getX(), results[i].getY());
        }

        double distanceToZeroX = boundingPolygon.getCentroid().x;
        double distanceToZeroY = boundingPolygon.getCentroid().y;

        boundingPolygon.translate( -distanceToZeroX, -distanceToZeroY );
        boundingPolygon.scale(1.5);
        boundingPolygon.translate( distanceToZeroX, distanceToZeroY );
        render.addBounding( boundingPolygon );
    }

    public void update( float delta )
    {
        this.delta = delta;

        sites = core.getSites();
        core.iterateSimple();
        setClusterNodes();

//        calculatePos();
//        calculateForces();
//        setSiteNodes();
//        core.voroDiagram();

        getDelaunay();

        render.addSites( core.getSites() );
    }

    private void getDelaunay()
    {
        for( int i = 0; i < neighbours.length; i++ )
        {
            for( int j = 0; j < neighbours[i].length; j++ )
            {
                neighbours[i][j] = false;
            }
        }
        d_edges.clear();
        for( Site s : sites )
        {
            for( Site s2 : s.getNeighbours())
            {
                d_edges.add(new DelaunayEdge( clusters.get(s.getIndex()), clusters.get(s2.getIndex())));
                neighbours[s.getIndex()][s2.getIndex()] = true;
            }
        }

        render.addDelaunay(d_edges);
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
                    node.setSite(s);
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
