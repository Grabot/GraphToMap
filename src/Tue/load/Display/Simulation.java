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
    private ArrayList<TestEdge> t_edges = new ArrayList<TestEdge>();
    private boolean[][] neighbours;
    private boolean clustererrors = false;
    private boolean cluster1Pos = false;
    private boolean clusterNodesPos = false;

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

    private double graphScaling = 0;

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
        this.graphScaling = display.graphScaling;
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

        forceMove = new ForceDirectedMovement( boundingPolygon, clusters, clusteredges, d_edges );

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

    private void updateCore()
    {
        core.adaptWeightsSimple();
        core.voroDiagram();
        render.addSites( core.getSites() );
        checkImage();
    }

    public void update( float delta )
    {
        //calculate the area's and apply them

        updateCore();
        while( !clustererrors )
        {
            iterations++;
            //applying force and do movement
            forceMove.ForceMoveCluster(delta);
            core.moveSitesBackCluster(clusters);
            updateCore();
            //calculate the area's and apply them
            //distortionmetric();
            checkError();
        }
        updateCore();
        if( !clusterNodesPos )
        {
            clusterNodesPos = true;
            System.out.println("iterations: " + iterations );
            positionClusterNodesFinal();
            //createTestEdges();

            //positionNodesRandom();
            //clusterVoronoiInit();
            positionNodeTest2();
        }
        else
        {
            //positionNodeTest2();
            //getTestEdgeForces( delta );
            //positionNodeTest();
            //clusterVoronoi( delta );
        }
    }

    private void positionClusterNodesFinal()
    {
        for( Cluster c : clusters )
        {
            PolygonSimple p = c.getSite().getPolygon();
            c.setPos( new Vector2(p.getCentroid().getX(), p.getCentroid().getY()));
        }
    }

    private void createTestEdges()
    {
        Cluster cl = clusters.get(3);
        for( Node n1 : cl.getNodes()) {
            double[] nodeToCluster = new double[clusterD.length];

            for (Cluster c : clusters) {
                TestEdge e = new TestEdge(n1, c, forces);
                t_edges.add(e);
                double total = 0;
                int amount = 0;
                for (Node n2 : c.getNodes()) {
                    amount++;
                    total = (total + pairD[n1.getIndex()][n2.getIndex()]);
                }
                if ((total / amount) == 0) {
                    nodeToCluster[c.getNumber()] = 0.01;
                } else {
                    nodeToCluster[c.getNumber()] = (total / amount);
                }
                total = 0;
                amount = 0;
            }

            for (TestEdge e : t_edges) {
                Cluster c = e.getDest();
                e.setWeight((nodeToCluster[c.getNumber()]*130));
            }
        }
        render.setNormalNodes(nodes);
    }

    private void getTestEdgeForces( float delta )
    {
        for( Node n : nodes )
        {
            n.setForce( new Vector2(0, 0));
        }

        for( TestEdge e : t_edges )
        {
            e.ApplyForces();
        }
        render.addTestEdge(t_edges);

        for( Node n : clusters.get(3).getNodes()) {
            double xMove = (n.getForce()).getX() * delta;
            double yMove = (n.getForce()).getY() * delta;

            n.setPos(new Vector2(n.getPos().getX() + xMove, n.getPos().y + yMove));
        }
    }

    private void positionNodeTest2() {

        Cluster cl = clusters.get(3);
        Node n1 = cl.getNodes().get(0);
        PolygonSimple poly = cl.getSite().getPolygon();
        double[] nodeToCluster = new double[clusterD.length];

        for (Cluster c : clusters) {
            double total = 0;
            int amount = 0;
            for (Node n2 : c.getNodes()) {
                amount++;
                total = (total + pairD[n1.getIndex()][n2.getIndex()]);
            }
            if ((total / amount) == 0) {
                nodeToCluster[c.getNumber()] = 0.01;
            } else {
                nodeToCluster[c.getNumber()] = (total / amount);
            }
            total = 0;
            amount = 0;
        }

        for (int i = 0; i < nodeToCluster.length; i++) {
            System.out.println(nodeToCluster[i]);
        }

        double xPos = rand.nextDouble() * 1200;
        double yPos = rand.nextDouble() * 800;

        n1.setPos(new Vector2(xPos, yPos));
        render.setNormalNodes(nodes);

        beaconBasedPositioning( n1, nodeToCluster );

        clusterNodesPos = true;
    }

    private void beaconBasedPositioning(Node n, double[] nodeToCluster )
    {
        double[] distances = new double[nodeToCluster.length];
        double lambda = 0.1;

        for( int i = 0; i < 10000; i++ ) {
            for (int j = 0; j < nodeToCluster.length; j++) {
                distances[j] = nodeToCluster[j] * lambda;
            }
            lambda = (lambda + 0.1);
            if( circleIntersectionCheck(distances) )
            {
                System.out.println("test");
                break;
            }
        }
        render.addNodToClusterTest(distances);
    }

    private boolean circleIntersectionCheck( double[] nodeToClusters )
    {
        boolean intersects = true;

        double radiusR1 = 0;
        double radiusR2 = 0;
        double distance = 0;

        for( Cluster c1 : clusters ) {
            for( Cluster c2 : clusters )
            {
                radiusR1 = nodeToClusters[c1.getNumber()];
                radiusR2 = nodeToClusters[c2.getNumber()];
                distance = c1.getPos().distance(c2.getPos());

                if( distance > (radiusR1+radiusR2))
                {
                    intersects = false;
                }
            }
        }

        return intersects;
    }

    private double getDistortionNode(double xPos, double yPos, double[] nodeToCluster )
    {
        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        double contractiontotal = 0;
        double contractionlocal = 0;
        double expansiontotal = 0;
        double expansionlocal = 0;
        int total = 0;

        double[] mapping = new double[clusterD.length];

        for (int i = 0; i < clusterD.length; i++) {
            Vector2 node1 = new Vector2(xPos, yPos);
            for (int j = 0; j < clusterD.length; j++)
            {
                Vector2 node2 = clusters.get(j).getPos();
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[j] = node1.distance(node2);
            }
        }

        System.out.println("");
        for( int i = 0; i < mapping.length; i++ )
        {
            System.out.println( mapping[i] );
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            total++;
            contractionlocal = (nodeToCluster[i] / mapping[i]);
            expansionlocal = (mapping[i] / nodeToCluster[i]);

            contractiontotal = (contractiontotal + contractionlocal);
            expansiontotal = (expansiontotal + expansionlocal);
        }

        contraction = (contractiontotal/total);
        expansion = (expansiontotal/total);
        distortion = (contraction*expansion);

        return distortion;
    }

    private void clusterVoronoi( float delta )
    {

        for( int i = 0; i < clusterD.length; i++ ) {
            //calculate the area's and apply them
            forceMove.ForceMoveNormal(delta);
            coreCluster[i].moveSitesBackNormal(clusters.get(i).getNodes());

            //calculate the area's and apply them
            coreCluster[i].adaptWeightsSimple();
            coreCluster[i].voroDiagram();

            render.addSites2(coreCluster[i].getSites(), i);
        }
    }

    private void clusterVoronoiInit()
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
                site.setPercentage(n.getWeight());
                site.setWeight(n.getWeight());
                n.setSite(site);
                sitesCluster[i].add(site);
            }

            coreCluster[i].normalizeSites(sitesCluster[i]);
            coreCluster[i].setSites(sitesCluster[i]);
            coreCluster[i].setClipPolygon(boundingCluster[i]);
            coreCluster[i].voroDiagram();
            coreCluster[i].setOldPoint();
            coreCluster[i].moveSitesBackNormal(clusters.get(i).getNodes());
        }

        forceMove = new ForceDirectedMovement( nodes, edges );

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
            System.out.println("all area's low error");
        }
        doneclusters = 0;
    }

    private void positionNodesRandom()
    {
        double xPos = 0;
        double yPos = 0;

        PolygonSimple poly = null;

        for( Cluster c : clusters )
        {
            poly = c.getSite().getPolygon();
            for( Node n : c.getNodes() )
            {
                xPos = rand.nextDouble()*width;
                yPos = rand.nextDouble()*height;

                while( !pnpoly(poly.length, poly.getXPoints(), poly.getYPoints(), xPos, yPos ))
                {
                    xPos = rand.nextDouble()*width;
                    yPos = rand.nextDouble()*height;
                }
                n.setPos(new Vector2( xPos, yPos ));
            }
        }

        render.setNormalNodes(nodes);
        render.setNormalEdges(edges);
        clusterNodesPos = true;
    }

    //https://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html
    //check to see whether point is inside polygon
    //nvert is number of sides to the polygon, vertx and verty are the x and y coordinates of the polygon
    //testx and testy are the x and y coordinates of the point you want to check. It returns true if true false otherwise
    private boolean pnpoly(int nvert, double[] vertx, double[] verty, double testx, double testy)
    {
        int i, j = 0;
        boolean c = false;

        for (i = 0, j = nvert-1; i < nvert; j = i++)
        {
            if ( ((verty[i]>testy) != (verty[j]>testy)) && (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
            {
                c = !c;
            }
        }
        return c;
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
