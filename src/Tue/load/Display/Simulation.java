package Tue.load.Display;

import Tue.load.Forces.Force;
import Tue.load.Geometry.ConvexHull;
import Tue.load.Geometry.VoronoiCore;
import Tue.load.PointPlacement;
import Tue.load.Positioning;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Simulation
{

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<DelaunayFace> d_faces = new ArrayList<DelaunayFace>();
    private ArrayList<Vector2> borderpoints = new ArrayList<Vector2>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private boolean[][] neighbours;
    private boolean clustererrors = false;
    private boolean cluster1Pos = false;
    private boolean clusterNodesPos = false;
    private float delta = 0;

    private int width;
    private int height;
    private double radiusGlobal = 10;
    private double stepSize = 0.1;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private OpenList sites2;
    private VoronoiCore core2;

    private OpenList[] sitesCluster;
    private VoronoiCore[] coreCluster;

    private Renderer render;
    private Force forces;
    private Random rand;

    private double[][] clusterD;
    private double[][] pairD;

    private int iterations = 0;

    private ConvexHull boundary;
    private ForceDirectedMovement forceMove;
    private ForceDirectedMovement forceMove2;
    private PointPlacement points;

    private Display display;

    private boolean firstImage = true;
    private boolean secondImage = false;

    public Simulation(Display display, Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height, Force forces, double[][] pairD, double[][] clusterD, PointPlacement points, ArrayList<Node> nodes, ArrayList<Edge> edges )
    {
        this.width = width;
        this.height = height;

        this.clusters = clusters;
        this.clusterD = clusterD;
        this.pairD = pairD;
        this.render = render;
        this.display = display;
        this.forces = forces;
        this.nodes = nodes;
        this.edges = edges;
        this.points = points;
        rand = new Random();

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

        ArrayList<DelaunayEdge> d_edges = getDelaunay();
        render.addDelaunay(d_edges);

        forceMove = new ForceDirectedMovement( boundingPolygon, clusters, clusteredges, d_edges, core );

        sites2 = new OpenList();
        core2 = new VoronoiCore();

        sitesCluster = new OpenList[clusterD.length];
        coreCluster = new VoronoiCore[clusterD.length];
    }

    private void ellipseBorder()
    {
        boundingPolygon = new PolygonSimple();

        int numPoints = 20;
        for (int j = 0; j < numPoints; j++)
        {
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

    private void checkImage()
    {
        if( firstImage )
        {
            writeImage("3withDelaunayInitial.png");
            firstImage = false;
        }
        else if( secondImage )
        {
            writeImage("3withDelaunaylowError.png");
            secondImage = false;
        }
    }

    private void writeImage(String fileName)
    {
        try
        {
            BufferedImage test = display.createImage();
            File outputfile = new File(fileName);
            ImageIO.write(test, "png", outputfile);
        }
        catch(Exception e)
        {
            System.out.println("error: " + e );
        }
    }

    public void update( float delta )
    {
        this.delta = delta;
        //calculate the area's and apply them

        core.adaptWeightsSimple();
        core.voroDiagram();
        render.addSites( core.getSites() );
        checkImage();


        while( !clustererrors )
        {
            iterations++;
            //applying force and do movement
            forceMove.ForceMove(delta);

            //calculate the area's and apply them
            core.adaptWeightsSimple();
            core.voroDiagram();

            render.addSites( core.getSites() );
            //distortionmetric();
            checkError();
        }

        if( !clusterNodesPos )
        {
            System.out.println("iterations: " + iterations );
            positionNodes();
            clusterVoronoiInitTest();
        }
        else
        {
            clusterVoronoiTest();
            setSiteClusterTest();
        }


        checkImage();
    }

    private void clusterVoronoi()
    {
        //calculate the area's and apply them
        core2.iterateSimple();

        render.addSites2( core2.getSites(), 0 );
    }

    private void setSiteClusterTest()
    {
        for( int i = 0; i < clusterD.length; i++ )
        {
            for( Node n : clusters.get(i).getNodes() )
            {
                n.setPos(new Vector2(n.getSite().getX(), n.getSite().getY()));
            }
        }
    }

    private void clusterVoronoiTest()
    {
        for( int i = 0; i < clusterD.length; i++ ) {
            //calculate the area's and apply them
            coreCluster[i].iterateSimple();

            render.addSites2(coreCluster[i].getSites(), i);
        }
    }

    private void clusterVoronoiInitTest()
    {
        PolygonSimple[] boundingCluster = new PolygonSimple[clusterD.length];
        for( int i = 0; i < clusterD.length; i++ ) {
            boundingCluster[i] = clusters.get(i).getSite().getPolygon();
        }

        for( int i = 0; i < clusterD.length; i++ ) {
            sitesCluster[i] = new OpenList();
            coreCluster[i] = new VoronoiCore();

            for (Node n : clusters.get(i).getNodes()) {
                Site site = new Site(n.getPos().getX(), n.getPos().getY());
                n.setSite(site);
                sitesCluster[i].add(site);
            }

            coreCluster[i].normalizeSites(sitesCluster[i]);

            coreCluster[i].setSites(sitesCluster[i]);
            coreCluster[i].setClipPolygon(boundingCluster[i]);
            coreCluster[i].voroDiagram();
            coreCluster[i].setOldPoint();
        }

    }

    private void clusterVoronoiInit()
    {
        PolygonSimple boundingcluster = clusters.get(0).getSite().getPolygon();

        sites2 = new OpenList();
        core2 = new VoronoiCore();

        for( Node n : clusters.get(0).getNodes() )
        {
            Site site = new Site( n.getPos().getX(), n.getPos().getY());
            sites2.add(site);
        }

        core2.normalizeSites(sites2);

        core2.setSites(sites2);
        core2.setClipPolygon(boundingcluster);
        core2.voroDiagram();
        core2.setOldPoint();

    }

    private void checkError()
    {
        double areaHave = 1;
        double areaWant = 1;
        double error = 1;
        int doneclusters = 0;
        for( Cluster c : clusters )
        {
            Site s = c.getSite();
            PolygonSimple p = s.getPolygon();
            areaHave = p.getArea();
            areaWant = boundingPolygon.getArea()*s.getPercentage();
            error = Math.abs(areaWant - areaHave) / (areaWant);

            if( error < 0.011 )
            {
                doneclusters++;
            }
        }
        if( doneclusters == clusters.size() )
        {
            clustererrors = true;
            secondImage = true;
            System.out.println("all area's low error with lower than 0.011 error for all area's");
        }
        doneclusters = 0;
    }

    private void positionNodes()
    {
        points.positionNodes();

        render.setNormalNodes(nodes);
        render.setNormalEdges(edges);
        clusterNodesPos = true;
    }

    private ArrayList<DelaunayEdge> getDelaunay()
    {
        ArrayList<DelaunayEdge> d_edges = new ArrayList<DelaunayEdge>();
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
                DelaunayEdge e = new DelaunayEdge( clusters.get(s.getIndex()), clusters.get(s2.getIndex()), forces );
                d_edges.add(e);
                neighbours[s.getIndex()][s2.getIndex()] = true;
            }
        }

        double[] increase = new double[d_edges.size()];
        for( int i = 0; i < d_edges.size(); i++ )
        {
            double distance = clusterD[d_edges.get(i).getSource().getNumber()][d_edges.get(i).getDest().getNumber()];
            double weight = d_edges.get(i).getSource().getPos().distance(d_edges.get(i).getDest().getPos());
            increase[i] = (distance/weight);
        }

        double average = 0;
        for( int i = 0; i < increase.length; i++ )
        {
            average = (average + increase[i]);
        }

        average = (average/increase.length);
        for( int i = 0; i < increase.length; i++ )
        {
            increase[i] = ((increase[i] - average) + 1);
        }

        for( int i = 0; i < increase.length; i++ )
        {
            double weight = d_edges.get(i).getSource().getPos().distance(d_edges.get(i).getDest().getPos());
            weight = (weight*increase[i]);
            d_edges.get(i).setWeight(weight);
        }

        return d_edges;
    }

}
