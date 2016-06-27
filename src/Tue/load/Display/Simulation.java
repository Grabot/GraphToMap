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
    private boolean clusterNodesPos = false;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private OpenList[] sitesCluster;
    private VoronoiCore[] coreCluster;

    private Renderer render;
    private Force forces;

    private double[][] clusterD;
    private double[][] pairD;

    private int iterations = 0;
    private int normalIterations = 0;

    private ConvexHull boundary;
    private ForceDirectedMovement forceMove;
    private PointPlacement points;

    private Display display;

    private boolean firstImage = true;
    private boolean secondImage = false;

    private BeaconPositioning beacon;

    private boolean normalNodePos = false;
    private boolean lastPositioning = true;
    private boolean randomPos = false;

    private double graphScaling;
    private double divideScale = 50;
    private double step = 100;

    public Simulation(Display display, Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height, Force forces, double[][] pairD, double[][] clusterD, PointPlacement points, ArrayList<Node> nodes, ArrayList<Edge> edges, double graphScaling )
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
        this.graphScaling = graphScaling;

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

        sitesCluster = new OpenList[clusterD.length];
        coreCluster = new VoronoiCore[clusterD.length];

        beacon = new BeaconPositioning(clusters, nodes);
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

    private void clusterPositioning( float delta )
    {
        iterations++;
        //applying force and do movement
        forceMove.ForceMoveCluster(delta);

        core.moveSitesBackCluster(clusters);
        core.adaptWeightsSimple();
        core.voroDiagram();

        render.addSites( core.getSites() );
        checkImage();
        //calculate the area's and apply them
        //distortionmetric();
        checkError();
    }

    private void NodePlacementInit2() {
        //clusterNodesPos = true;
            beacon.positionClusterNodesFinal();

            double[] nodeToCluster = new double[clusterD.length];

            Cluster cl = clusters.get(3);
            Node n = cl.getNodes().get(0);
    //        for( Cluster cl : clusters )
    //        {
    //            for( Node n : cl.getNodes() )
    //            {
            //this will find all intersecting points of any 2 circles and save them, later it will check if they engulf all points

            for (Cluster c : clusters) {
                double total = 0;
                int amount = 0;
                for (Node n2 : c.getNodes()) {
                    amount++;
                    total = (total + pairD[n.getIndex()][n2.getIndex()]);
                }
                if ((total / amount) == 0) {
                    nodeToCluster[c.getNumber()] = 0.01;
                } else {
                    nodeToCluster[c.getNumber()] = (total / amount);
                }
                total = 0;
                amount = 0;
            }
        //set the start position for the node
//                n.setPos( new Vector2(0, 0));
//
//                boolean nodePlaced = false;
//                while( !nodePlaced )
//                {
//                    nodePlaced = placeNode( n, nodeToCluster );
//                }

        n.setPos(new Vector2(0, 0));
        render.setBeaconBasedTest(nodeToCluster, n);
//            }
//            scaleInCluster( cl );
//            beacon.scaleToCluster( cl );
//        }
//        clusterVoronoiInit();
//
//        if( clusterNodesPos ) {
//            beacon.finalPositioningCheck();
//        }
//
//        render.setNormalNodes( nodes );
//        render.setNormalEdges( edges );
    }

    private void scaleInCluster( Cluster cl )
    {
        PolygonSimple poly = cl.getSite().getPolygon();
        Vector2 polyCenter = new Vector2( poly.getCentroid().getX(), poly.getCentroid().getY() );
        double closest = 9999;
        double furthest = 0;
        double distance = 0;
        //first find the one furthest away and the one closest.
        //Put the furthest on the edge and the closest on the center, the rest in between accordingly
        for( Node n : cl.getNodes() )
        {
            distance = polyCenter.distance( n.getPos() );
            if( distance > furthest )
            {
                furthest = distance;
            }
            if( distance < closest )
            {
                closest = distance;
            }
        }

        //System.out.println("cluster: " + cl.getNumber() + " furthest: " + furthest + " closest: " + closest );
    }

    private boolean placeNode( Node n, double[] nodeToCluster )
    {
        Vector2 newPos = setNodePos( getDistortionNodeScale( n.getPos().getX(), n.getPos().getY(), nodeToCluster ), n, nodeToCluster );
        if( !(newPos.getX() == 0 && newPos.getY() == 0) )
        {
            step = (step*2);
            n.setPos(newPos);
        }
        else
        {
            step = (step/2);
        }

        if( step < 0.01 )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private Vector2 setNodePos( double distortionInit, Node n, double[] nodeToCluster )
    {
        //get a certain amount of samples around the node
        int numPoints = 360;
        double distortion = 0;
        double lowest = 9999;
        Vector2 newPos = new Vector2(0, 0);

        for (int i = 0; i < numPoints; i++)
        {
            double angle = 2.0 * Math.PI * (i * 1.0 / numPoints);
            double rotate = 2.0 * Math.PI / numPoints / 2;
            double x = n.getPos().getX() + Math.cos(angle + rotate) * (step);
            double y = n.getPos().getY() + Math.sin(angle + rotate) * (step);

            distortion = getDistortionNodeScale( x, y, nodeToCluster );
            if((distortion < lowest) && (distortion < distortionInit) )
            {
                lowest = distortion;
                newPos = new Vector2( x ,y );
            }
        }

        return newPos;
    }

    private double getDistortionNodeScale( double xPos, double yPos, double[] nodeToCluster )
    {
        double distortion = 0;

        double contractionlocal = 0;
        double expansionlocal = 0;
        int total = 0;

        double[] mapping = new double[clusters.size()];

        for (int i = 0; i < clusters.size(); i++) {
            Vector2 node1 = new Vector2(xPos, yPos);
            for (int j = 0; j < clusters.size(); j++)
            {
                Vector2 node2 = clusters.get(j).getPos();
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[j] = node1.distance(node2);
            }
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            total++;
            contractionlocal = (((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]) / mapping[i]);
            expansionlocal = (mapping[i] / ((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]));

            if( contractionlocal >= expansionlocal )
            {
                distortion += contractionlocal;
            }
            else if( expansionlocal >= contractionlocal )
            {
                distortion += expansionlocal;
            }
        }
        distortion = (distortion/total);

        return distortion;
    }

    private void NodePlacementInit()
    {
        clusterNodesPos = true;
        System.out.println("iterations: " + iterations );
        beacon.positionClusterNodesFinal();

        if( randomPos ) {
            beacon.positionNodesRandom( width, height );
        }
        else {
            beacon.beacondBasedPositioning(clusterD, pairD);
        }
        clusterVoronoiInit();

        if( clusterNodesPos ) {
            beacon.finalPositioningCheck();
        }
    }


    private void NodePlacementVoronoi( float delta )
    {

        for( int i = 0; i < clusterD.length; i++ ) {
            if( clusters.get(i).getNodes().size() != 1 ) {
                //calculate the area's and apply them
                if (!forceMove.getMovement()) {
                    forceMove.ForceMoveNormal(delta);
                }
                coreCluster[i].moveSitesBackNormal(clusters.get(i).getNodes());

                //calculate the area's and apply them
                coreCluster[i].adaptWeightsSimple();
                coreCluster[i].voroDiagram();

                render.addSites2(coreCluster[i].getSites(), i);
            }
        }
    }

    boolean test = true;
    public void update( float delta, boolean iterate, boolean demo )
    {
        //calculate the area's and apply them
        if( iterate )
        {
            if( !clustererrors )
            {
                if( demo )
                {
                    for( int i = 0; i < (iterations/400); i++ )
                    {
                        clusterPositioning( delta );
                    }
                }
                clusterPositioning( delta );
            }
            else if( !clusterNodesPos )
            {
                NodePlacementInit2();
                if( clusterNodesPos ) {
                    render.setNormalNodes(nodes);
                    render.setNormalEdges(edges);
                }
            }
            else if( !normalNodePos )
            {
                normalIterations++;
                NodePlacementVoronoi( delta );
                checkErrorNormal();
            }
        }
        else
        {
            while( !clustererrors )
            {
                clusterPositioning( delta );
            }

            if( !normalNodePos )
            {
                normalIterations++;
                if( !clusterNodesPos )
                {
                    if( test ) {
                        NodePlacementInit2();
                        test = false;
                    }
                }
                else
                {
                    NodePlacementVoronoi(delta);
                    checkErrorNormal();
                }
                render.setNormalNodes(nodes);
                render.setNormalEdges(edges);
            }
        }

        if( lastPositioning && normalNodePos )
        {
            System.out.println("normal nodes iterations: " + normalIterations );
            lastPositioning = false;
            beacon.finalNodePositioning();
            render.setNormalNodes(nodes);
            render.setNormalEdges(edges);
        }
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

    private void clusterVoronoiInit()
    {
        randomPos = false;
        PolygonSimple[] boundingCluster = new PolygonSimple[clusterD.length];
        for( int i = 0; i < clusterD.length; i++ ) {
            boundingCluster[i] = clusters.get(i).getSite().getPolygon();
        }

        for( int i = 0; i < clusterD.length; i++ ) {
            if( clusters.get(i).getNodes().size() != 1) {
                sitesCluster[i] = new OpenList();
                coreCluster[i] = new VoronoiCore();

                for (Node n : clusters.get(i).getNodes()) {
                    Site site = new Site(n.getPos().getX(), n.getPos().getY());
                    site.setPercentage(n.getWeight());
                    site.setWeight(n.getWeight());
                    n.setSite(site);
                    sitesCluster[i].add(site);
                }

                try {
                    coreCluster[i].normalizeSites(sitesCluster[i]);
                    coreCluster[i].setSites(sitesCluster[i]);
                    coreCluster[i].setClipPolygon(boundingCluster[i]);
                    coreCluster[i].voroDiagram();
                    coreCluster[i].setOldPoint();
                    coreCluster[i].moveSitesBackNormal(clusters.get(i).getNodes());
                } catch (Exception e) {
                    clusterNodesPos = false;
                    randomPos = true;
                    System.out.println("error: " + e);
                    break;
                }
            }
        }

        if( !randomPos ) {
            forceMove = new ForceDirectedMovement(nodes, edges, clusters);
        }

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
            if( p == null )
            {
                error = 100;
            }
            else {
                areaHave = p.getArea();
                areaWant = boundingPolygon.getArea() * s.getPercentage();
                error = Math.abs(areaWant - areaHave) / (areaWant);
            }
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

        //hard exit on the cluster resizing
        if( iterations >= 200000 )
        {
            clustererrors = true;
            secondImage = true;
            System.out.println("all area's low error");
        }
        doneclusters = 0;
    }


    private void checkErrorNormal()
    {
        double areaHave = 1;
        double areaWant = 1;
        double error = 1;
        int doneNodes = 0;
        for( Cluster c : clusters )
        {
            if( c.getNodes().size() != 1 ) {
                for (Node n : c.getNodes()) {
                    Site s = n.getSite();
                    PolygonSimple p = s.getPolygon();
                    if (p == null) {
                        error = 100;
                    } else {
                        areaHave = p.getArea();
                        areaWant = c.getSite().getPolygon().getArea() * s.getPercentage();
                        error = Math.abs(areaWant - areaHave) / (areaWant);
                    }
                    if (error < 0.011) {
                        doneNodes++;
                    }
                }
            }
            else
            {
                doneNodes++;
            }
        }

        if( doneNodes == nodes.size() )
        {
            normalNodePos = true;
            System.out.println("all normal node area's low error");
        }

        if( normalIterations >= 10000 )
        {
            normalNodePos = true;
            System.out.println("all normal node area's low error");
        }

        if(( normalIterations % 100 ) == 0 )
        {
            System.out.println("iterations: " + normalIterations );
        }
        doneNodes = 0;
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
