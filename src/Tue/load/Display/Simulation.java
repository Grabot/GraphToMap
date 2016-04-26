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

    private Renderer render;
    private Force forces;
    private Random rand;

    private double[][] clusterD;
    private double[][] pairD;

    private int iterations = 0;

    private ConvexHull boundary;
    private ForceDirectedMovement forceMove;
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


        checkImage();
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
        double[][] nodeToCluster = new double[pairD.length][clusterD.length];

        for( int i = 0; i < nodes.size(); i++ )
        {
            for( Cluster c : clusters )
            {
                double total = 0;
                int amount = 0;
                for( Node n : c.getNodes() )
                {
                    amount++;
                    total = (total + pairD[i][n.getIndex()]);
                }
                if((total/amount) == 0)
                {
                    nodeToCluster[i][c.getNumber()] = 0.01;
                }
                else {
                    nodeToCluster[i][c.getNumber()] = (total / amount);
                }
                total = 0;
                amount = 0;
            }
        }

        for( Cluster c : clusters )
        {
            Color color = new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
            for (Node n : c.getNodes())
            {
                SimulatedAnnealingNode(c, n, nodeToCluster);
                n.setPos(new Vector2( normalP[99].getNormalPos()[0], normalP[99].getNormalPos()[1] ));
                n.setColor(color);
            }
        }

        render.setNormalNodes(nodes);
        render.setNormalEdges(edges);
        clusterNodesPos = true;
    }

    private Positioning[] normalP = new Positioning[100];
    private ArrayList<double[]> positions = new ArrayList<double[]>();
    private void SimulatedAnnealingNode( Cluster c, Node n, double[][] nodeToCluster )
    {
        double[]empty = null;
        for( int i = 0; i < normalP.length; i++ )
        {
            Positioning pos = new Positioning( empty, Double.MAX_VALUE );
            normalP[i] = pos;
        }
        positions.clear();

        double xPos;
        double yPos;
        PolygonSimple p = c.getSite().getPolygon();

        for( int i = 0; i < 1000; i++ )
        {
            xPos = rand.nextDouble()*width;
            yPos = rand.nextDouble()*height;

            while( !(pnpoly(p.length, p.getXPoints(), p.getYPoints(), xPos, yPos )))
            {
                xPos = rand.nextDouble() * width;
                yPos = rand.nextDouble() * height;
            }

            double[] pos = new double[2];
            pos[0] = xPos;
            pos[1] = yPos;

            positions.add(pos);
        }

        getDistortionNormal(c, n, nodeToCluster);
        while( radiusGlobal > 0.1 )
        {
            getDistortionNormal(c, n, nodeToCluster);
            radiusGlobal=(radiusGlobal-stepSize);
        }


    }

    private void getDistortionNormal( Cluster c, Node n, double[][] nodeToCluster )
    {
        //the maximum of the actual distance divided with the mapping distance
        double contraction = 0;
        //the maximum of the mapping distance divided with the actual distance
        double expansion = 0;
        //distortion is the multiplication of the 2. The ideal would be a distortion of 1
        double distortion;

        double contractiontotal = 0;
        double expansiontotal = 0;
        int total = 0;

        for( double[] nodePos : positions )
        {
            double[] mapping = new double[clusterD.length];

            for (int i = 0; i < clusterD.length; i++) {
                Vector2 node1 = new Vector2(nodePos[0], nodePos[1]);
                for (int j = 0; j < clusterD.length; j++)
                {
                    Vector2 node2 = clusters.get(j).getPos();
                    //get the actual distances between all nodes to calculate the distorion metrics
                    mapping[j] = node1.distance(node2);
                }
            }

            for( int i = 0; i < mapping.length; i++ )
            {
                total++;
                double contractionlocal = (nodeToCluster[n.getIndex()][i] / mapping[i]);
                double expansionlocal = (mapping[i] / nodeToCluster[n.getIndex()][i]);

                contractiontotal = (contractiontotal + contractionlocal);
                expansiontotal = (expansiontotal + expansionlocal);
            }

            contraction = (contractiontotal/total);
            expansion = (expansiontotal/total);
            distortion = (contraction*expansion);


            contraction = 0;
            expansion = 0;
            contractiontotal = 0;
            expansiontotal = 0;
            total = 0;

            if( normalP[0].getDistortion() > distortion )
            {
                Positioning pos = new Positioning( nodePos, distortion );
                normalP[0] = pos;
            }

            sortNormal();
        }

        getNewPointsNormal( c.getSite().getPolygon() );
    }

    private void getNewPointsNormal( PolygonSimple p )
    {
        positions.clear();

        double radius;
        double degree;
        double x;
        double y;
        double newX = 0;
        double newY = 0;

        for( int i = 0; i < normalP.length; i++ )
        {
            for( int j = 0; j < 10; j++ )
            {
                double[] pos = normalP[i].getNormalPos();
                double[] newPos = new double[2];
                x = pos[0];
                y = pos[1];
                degree = ((rand.nextDouble()*360)*(Math.PI / 180));
                radius = rand.nextDouble()*radiusGlobal;
                newX = (x + radius*Math.sin(degree));
                newY = (y + radius*Math.cos(degree));

                while( !(pnpoly(p.length, p.getXPoints(), p.getYPoints(), newX, newY )))
                {
                    //get a random angle and a random length to find the next point placement.
                    degree = ((rand.nextDouble()*360)*(Math.PI / 180));
                    radius = rand.nextDouble()*radiusGlobal;
                    newX = (x + radius*Math.sin(degree));
                    newY = (y + radius*Math.cos(degree));
                }
                newPos[0] = newX;
                newPos[1] = newY;
                positions.add(newPos);
            }
        }
    }

    private void sortNormal()
    {
        int n = normalP.length;
        Positioning temp;
        for (int v = 1; v < n; v++) {
            if (normalP[v - 1].getDistortion() < normalP[v].getDistortion()) {
                temp = normalP[v - 1];
                normalP[v - 1] = normalP[v];
                normalP[v] = temp;
            }
        }

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
