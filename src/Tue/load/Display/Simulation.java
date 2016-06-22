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
    private int normalIterations = 0;

    private ConvexHull boundary;
    private ForceDirectedMovement forceMove;
    private ForceDirectedMovement forceMove2;
    private PointPlacement points;

    private Display display;

    private boolean firstImage = true;
    private boolean secondImage = false;

    private double graphScaling = 0;

    private boolean normalNodePos = false;
    private boolean randomPos = false;
    private Vector2 nodePosition = new Vector2(0, 0);

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

    private boolean lastPositioning = true;
    public void update( float delta )
    {
        //calculate the area's and apply them

        //updateCore();
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



        while( !normalNodePos )
        {
            normalIterations++;
            if( !clusterNodesPos )
            {
                clusterNodesPos = true;
                System.out.println("iterations: " + iterations );
                positionClusterNodesFinal();
                //createTestEdges();

                if( randomPos ) {
                    positionNodesRandom();
                }
                else {
                    beacondBasedPositioning();
                }
                clusterVoronoiInit();

                if( clusterNodesPos ) {
                    finalPositioningCheck();
                    render.setNormalNodes(nodes);
                    render.setNormalEdges(edges);
                }
            }
            else {
                clusterVoronoi(delta);
                checkErrorNormal();
            }
        }
        if( lastPositioning && normalNodePos  )
        {
            System.out.println("normal nodes iterations: " + normalIterations );
            lastPositioning = false;
            finalNodePositioning();
        }
    }

    public void updateIterative( float delta )
    {
        //calculate the area's and apply them

        if( !clustererrors )
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
        else if( !clusterNodesPos )
        {
            clusterNodesPos = true;
            System.out.println("iterations: " + iterations );
            positionClusterNodesFinal();
            //createTestEdges();

            if( randomPos ) {
                positionNodesRandom();
            }
            else {
                beacondBasedPositioning();
            }
            clusterVoronoiInit();
            //updateCore();

            if( clusterNodesPos ) {
                finalPositioningCheck();
                render.setNormalNodes(nodes);
                render.setNormalEdges(edges);
            }
        }
        else if( !normalNodePos )
        {
            normalIterations++;
            clusterVoronoi( delta );
            checkErrorNormal();
        }


        if( lastPositioning && normalNodePos )
        {
            System.out.println("normal nodes iterations: " + normalIterations );
            lastPositioning = false;
            finalNodePositioning();
        }
    }

    private void finalNodePositioning()
    {
        for( Cluster c : clusters )
        {
            if( c.getNodes().size() != 1 ) {
                for (Node n : c.getNodes()) {
                    PolygonSimple p = n.getSite().getPolygon();
                    n.setPos(new Vector2(p.getCentroid().getX(), p.getCentroid().getY()));
                }
            }
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


    private void beacondBasedPositioning() {
        //this will find all intersecting points of any 2 circles and save them, later it will check if they engulf all points
        for( Cluster cl : clusters )
        {
            for (Node n : cl.getNodes()) {
                double[] nodeToCluster = new double[clusterD.length];

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

                beaconIntersections(nodeToCluster);
                Vector2 foundPosition = new Vector2(nodePosition.x, nodePosition.y);
                nodes.get(n.getIndex()).setPos(foundPosition);
                System.out.println("found position node: " + n.getIndex() );
            }
            scaleToCluster(cl);
        }


    }

    private void finalPositioningCheck()
    {
        //it checks wheter 2 points are placed on top of each other, this messes up the voronoi creation.
        for( Node n1 : nodes )
        {
            for( Node n2 : nodes )
            {
                if( n1.getIndex() != n2.getIndex() )
                {
                    if( n1.getPos().equals(n2.getPos()))
                    {
                        //if this is the case, just move one a bit to the side.
                        n1.setPos(new Vector2((n1.getPos().getX()-0.1), (n1.getPos().getY()-0.1)));
                    }
                }
            }
        }
        singleClusterCheck();
    }

    private void singleClusterCheck()
    {
        //also if there is a cluster with only 1 node we will place that node in the cluster center
        for( Cluster c : clusters )
        {
            if( c.getNodes().size() == 1 )
            {
                Node n = c.getNodes().get(0);
                PolygonSimple p = c.getSite().getPolygon();
                n.setPos(new Vector2(p.getCentroid().getX(), p.getCentroid().getY() ));
                //we will also set the polygon for this node, since it won't have a site.
                n.setSite( c.getSite() );
            }
        }
    }

    private void beaconIntersections( double[] nodeToCluster )
    {
        double[] distances = new double[nodeToCluster.length];
        double lambda = 0.1;

        for( int i = 0; i < 100000; i++ ) {
            for (int j = 0; j < nodeToCluster.length; j++) {
                distances[j] = nodeToCluster[j] * lambda;
            }
            lambda = (lambda + 0.1);
            if( circleIntersectionCheck(distances) )
            {
                break;
            }
        }
    }

    private boolean circleIntersectionCheck( double[] nodeToClusters )
    {
        ArrayList<Vector2> circleIntersections = new ArrayList<Vector2>();
        boolean intersects = true;

        double radiusR1 = 0;
        double radiusR2 = 0;
        double distance = 0;

        for( Cluster c1 : clusters ) {
            for( Cluster c2 : clusters )
            {
                if(c1.getNumber() != c2.getNumber())
                {
                    radiusR1 = nodeToClusters[c1.getNumber()];
                    radiusR2 = nodeToClusters[c2.getNumber()];
                    distance = c1.getPos().distance(c2.getPos());

                    if (distance > (radiusR1 + radiusR2))
                    {
                        //they are separate and don't intersect
                    }
                    else if( distance < (radiusR1 - radiusR2 ))
                    {
                        //one circle is contained within the other.
                    }
                    else
                    {
                        //find intersection points of the 2 overlapping circles.
                        //http://paulbourke.net/geometry/circlesphere/
                        //we can find the intersection based on the triangle between c1, newPos and the intersection.
                        //there should be 2 points, so 2 solutions for x, y
                        double dx = c2.getPos().getX()-c1.getPos().getX();
                        double dy = c2.getPos().getY()-c1.getPos().getY();

                        double a = (((radiusR1*radiusR1)-(radiusR2*radiusR2) + (distance*distance))/(2*distance));

                        double newX = c1.getPos().getX() + (dx*(a/distance));
                        double newY = c1.getPos().getY() + (dy*(a/distance));

                        Vector2 P2 = new Vector2( newX, newY );

                        //determine the distance from point P2 to the 2 intersection points.
                        double h = Math.sqrt(((radiusR1*radiusR1) - (a*a)));

                        double rx = (-dy * (h/distance));
                        double ry = (dx * (h/distance));

                        double x3 = P2.getX()+rx;
                        double y3 = P2.getY()+ry;

                        double x3Prime = P2.getX()-rx;
                        double y3Prime = P2.getY()-ry;

                        circleIntersections.add(new Vector2(x3, y3));
                        circleIntersections.add(new Vector2(x3Prime, y3Prime));
                    }
                }
            }
        }

        intersects = checkPointsIntersection( nodeToClusters, circleIntersections );

        render.addNodeToClusterTest(nodeToClusters);
        render.addCircleTest(circleIntersections);
        return intersects;
    }

    private boolean checkPointsIntersection( double[] nodeToClusters, ArrayList<Vector2> circleIntersections )
    {
        if( circleIntersections.size() > 0 )
        {
            for( Vector2 point : circleIntersections )
            {
                if( !(Double.isNaN(point.getX()) && Double.isNaN(point.getY())) ) {
                    boolean intersects = true;
                    for (Cluster c : clusters) {
                        double radiusToPoint = point.distance(c.getPos());
                        double radiusCluster = nodeToClusters[c.getNumber()];

                        if (radiusCluster < radiusToPoint) {
                            intersects = false;
                        }
                    }
                    if (intersects) {
                        //there is a point which is in all the circle radii.
                        nodePosition = point;
                        return true;
                    }
                }
            }
        }
        else
        {
            return false;
        }
        return false;
    }

    private void scaleToCluster( Cluster cl )
    {
        //check if all the points are inside the cluster
        boolean inside = false;
        PolygonSimple p = cl.getSite().getPolygon();
        while( !inside ) {
            //we assume that all the nodes are inside, if that is the case we leave the loop
            inside = true;
            //check all the nodes of the cluster if they are inside the cluster.
            for (Node n : cl.getNodes()) {
                if (!pnpoly(p.getNumPoints(), p.getXPoints(), p.getYPoints(), n.getPos().getX(), n.getPos().getY())) {
                    //first mark it that it's not inside
                    inside = false;
                    //if a point is not inside, shrink the points till they do, this can be done for each node
                    Vector2 center = new Vector2(p.getCentroid().getX(), p.getCentroid().getY());
                    for (Node nMove : cl.getNodes()) {
                        //first translate it to the center, so the center of the cluster should be (0, 0)
                        n.setPos(new Vector2(n.getPos().getX() - center.getX(), n.getPos().getY() - center.getY()));
                        //scale it down
                        n.setPos(new Vector2(n.getPos().getX() * 0.99, n.getPos().getY() * 0.99));
                        //translate it back
                        n.setPos(new Vector2(n.getPos().getX() + center.getX(), n.getPos().getY() + center.getY()));
                    }
                }
            }
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

    private void clusterVoronoi( float delta )
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
}
