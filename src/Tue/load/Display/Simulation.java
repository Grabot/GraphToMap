package Tue.load.Display;

import Tue.load.Forces.Force;
import Tue.load.Geometry.ConvexHull;
import Tue.load.Geometry.VoronoiCore;
import Tue.load.PointPlacement;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.awt.*;
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

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;
    private Force forces;
    private Random rand;

    private double[][] clusterD;

    private int iterations = 0;

    private ConvexHull boundary;
    private ForceDirectedMovement forceMove;
    private PointPlacement points;

    public Simulation(Renderer render, ArrayList<Cluster> clusters, ArrayList<ClusterEdge> clusteredges, int width, int height, Force forces, double[][] clusterD, PointPlacement points, ArrayList<Node> nodes, ArrayList<Edge> edges )
    {
        this.width = width;
        this.height = height;

        this.clusters = clusters;
        this.clusterD = clusterD;
        this.render = render;
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

    public void update( float delta )
    {
        this.delta = delta;
        //calculate the area's and apply them

        if( !clustererrors )
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

        if( clustererrors )
        {
            if( !clusterNodesPos )
            {
                System.out.println("iterations: " + iterations );
                positionNodes();
            }
        }
        else
        {

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
            System.out.println("all area's low error with lower than 0.011 error for all area's");
        }
        doneclusters = 0;
    }

    private void positionCluster1()
    {
        ArrayList<Node> cluster1 = new ArrayList<Node>();
        Cluster c = clusters.get(0);
        PolygonSimple p = c.getSite().getPolygon();
        cluster1 = c.getNodes();
        boolean set = false;
        for( Node n : cluster1 )
        {
            while( !set ) {
                double xPos = rand.nextDouble() * width;
                double yPos = rand.nextDouble() * height;
                if (pnpoly(p.length, p.getXPoints(), p.getYPoints(), xPos, yPos)) {
                    n.setPos(new Vector2(xPos, yPos));
                    set = true;
                }
            }
            set = false;
        }
        cluster1Pos = true;
        render.setNormalNodes(nodes);
    }

    private void positionNodes()
    {
//        double[][] nodePos = points.defineNormalNodePosition();
//
//        clusterNodesPos = true;
//
//        //define all cluster nodes
//        for( int i = 0; i < nodes.size(); i++ )
//        {
//            nodes.get(i).setPos( new Vector2(nodePos[0][i], nodePos[1][i] ));
//        }
//
//        for( Cluster c : clusters )
//        {
//            Color color = new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
//            for( Node n : c.getNodes() )
//            {
//                n.setColor(color);
//            }
//        }
//
//        render.setNormalNodes(nodes);
//        render.setNormalEdges(edges);

        //points.PointPlacementNormal(clusters);
        for( Cluster c : clusters ) {
            ArrayList<Node> clusternodes;
            PolygonSimple p = c.getSite().getPolygon();
            clusternodes = c.getNodes();
            Color nodeColour = new Color( rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
            boolean set = false;
            for (Node n : clusternodes) {
                while (!set) {
                    double xPos = rand.nextDouble() * width;
                    double yPos = rand.nextDouble() * height;
                    if (pnpoly(p.length, p.getXPoints(), p.getYPoints(), xPos, yPos)) {
                        n.setPos(new Vector2(xPos, yPos));
                        n.setColor(nodeColour);
                        set = true;
                    }
                }
                set = false;
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

        double total = 0;
        for( int i = 0; i < increase.length; i++ )
        {
            total = (total + increase[i]);
        }

        total = (total/increase.length);
        for( int i = 0; i < increase.length; i++ )
        {
            increase[i] = ((increase[i] - total) + 1);
        }

        for( int i = 0; i < increase.length; i++ )
        {
            double weight = d_edges.get(i).getSource().getPos().distance(d_edges.get(i).getDest().getPos());
            weight = (weight*increase[i]);
            d_edges.get(i).setWeight(weight);
        }

        return d_edges;
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
