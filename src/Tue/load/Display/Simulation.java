package Tue.load.Display;

import Tue.load.Forces.Force;
import Tue.load.Geometry.ConvexHull;
import Tue.load.Geometry.VoronoiCore;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.util.ArrayList;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Simulation
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<DelaunayEdge> d_edges = new ArrayList<DelaunayEdge>();
    private ArrayList<DelaunayFace> d_faces = new ArrayList<DelaunayFace>();
    private ArrayList<Vector2> borderpoints = new ArrayList<Vector2>();
    private boolean[][] neighbours;
    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;
    private Force forces;

    private double[][] clusterD;

    private int iterations = 0;

    private ConvexHull boundary;

    private ForceDirectedMovement forceMove;
    private AreaMovement areaMove;

    public Simulation(Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height, Force forces, double[][] clusterD )
    {
        this.width = width;
        this.height = height;

        this.clusters = clusters;
        this.clusterD = clusterD;
        this.render = render;
        this.forces = forces;

        neighbours = new boolean[clusters.size()][clusters.size()];

        // normal list based on an array
        sites = new OpenList();
        core = new VoronoiCore();

        boundary = new ConvexHull();
        ellipseBorder();
        //convexBorder();

        for (int i=0;i<clusters.size();i++){
            Site site = new Site(clusters.get(i).getPos().x, clusters.get(i).getPos().y);
            //site.setPercentage(rand.nextFloat());
            site.setPercentage(clusters.get(i).getPercentage()*100);
            site.setIndex(clusters.get(i).getNumber());
            sites.add(site);
            borderpoints.add( new Vector2( clusters.get(i).getPos().x, clusters.get(i).getPos().y ));
            clusters.get(i).setSite(site);
        }

        core.normalizeSites(sites);

        core.setSites(sites);
        core.setClipPolygon(boundingPolygon);
        core.voroDiagram();
        core.setOldPoint();

        getDelaunay();
        render.addDelaunay(d_edges);

        forceMove = new ForceDirectedMovement( clusters, clusteredges, d_edges,  core );
        areaMove = new AreaMovement( clusters, core );
    }

    private void ellipseBorder()
    {
        boundingPolygon = new PolygonSimple();

        int numPoints = 40;
        for (int j = 0; j < numPoints; j++) {
            double angle = 2.0 * Math.PI * (j * 1.0 / numPoints);
            double rotate = 2.0 * Math.PI / numPoints / 2;
            double y = Math.sin(angle + rotate) * (height/2) + (height/2);
            double x = Math.cos(angle + rotate) * (width/2) + (width/2);
            boundingPolygon.add(x, y);
        }

        render.addBounding( boundingPolygon );
    }
    private void convexBorder()
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

        forceMove.ForceMove(delta);
        areaMove.AreaMove();

        render.addSites( core.getSites() );
        //distortionmetric();
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
                d_edges.add(new DelaunayEdge( clusters.get(s.getIndex()), clusters.get(s2.getIndex()), forces));
                neighbours[s.getIndex()][s2.getIndex()] = true;
            }
        }

        for( int i = 0; i < neighbours.length; i++ )
        {
            for( int j = (i+1); j < neighbours[i].length; j++ )
            {
                for( int k = j+1; k < neighbours[i].length; k++ )
                {
                    if( neighbours[i][j] && neighbours[i][k])
                    {
                        //if there are 2 neighbouring faces, check if those faces are also neighbouring
                        if( neighbours[j][k])
                        {
                            d_faces.add( new DelaunayFace(clusters.get(i), clusters.get(j), clusters.get(k)));
                        }
                    }
                }
            }
        }

        render.addDelaunay(d_edges);
    }

    private void distortionmetric()
    {
        double[][] pos = new double[2][clusterD.length];
        for( int i = 0; i < clusters.size(); i++ )
        {
            pos[0][i] = clusters.get(i).getPos().getX();
            pos[1][i] = clusters.get(i).getPos().getY();
        }

        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        double contractiontotal = 0;
        double expansiontotal = 0;
        int total = 0;

        double[][] mapping = new double[clusterD.length][clusterD.length];

        for (int i = 0; i < clusterD.length; i++) {
            Vector2 node1 = new Vector2(pos[0][i], pos[1][i]);
            for (int j = 0; j < clusterD.length; j++)
            {
                Vector2 node2 = new Vector2(pos[0][j], pos[1][j]);
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[i][j] = node1.distance(node2);
            }
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            for( int j = 0; j < mapping[i].length; j++ )
            {
                if( i != j )
                {
                    total++;
                    double contractionlocal = (clusterD[i][j] / mapping[i][j]);
                    double expansionlocal = (mapping[i][j] / clusterD[i][j]);

                    contractiontotal = (contractiontotal + contractionlocal);
                    expansiontotal = (expansiontotal + expansionlocal);
                }
            }
        }

        contraction = (contractiontotal/total);
        expansion = (expansiontotal/total);
        distortion = (contraction*expansion);

        contraction = 0;
        expansion = 0;
        contractiontotal = 0;
        expansiontotal = 0;
        total = 0;

        System.out.println("distortion: " + distortion);
    }
}
