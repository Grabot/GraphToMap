package Tue.load.Display;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Renderer
{
    private ArrayList<Cluster> clusternodes = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();
    private ArrayList<DelaunayEdge> d_edges =  new ArrayList<DelaunayEdge>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private ArrayList<TestEdge> t_edges = new ArrayList<TestEdge>();
    private OpenList sites;
    private OpenList[] siteCluster;

    private Color[] siteColours;

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;
    private PolygonSimple newBorder = null;
    private Random rand;

    private double zoom = 1;
    private Display display;

    private double movementX = 0;
    private double movementY = 0;

    public Renderer(Display display, ArrayList<Cluster> clusternodes, ArrayList<ClusterEdge> clusteredges)
    {
        this.display = display;
        rand = new Random();
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;

        sites = new OpenList();
        siteCluster = new OpenList[clusternodes.size()];
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            siteCluster[i] = new OpenList();
        }
        siteColours = new Color[clusternodes.size()];
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            siteColours[i] = new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
        }
    }

    private void checkMovement()
    {
        //check zooming
        g2.translate(display.getWidth()/2, display.getHeight()/2);
        g2.scale(zoom, zoom);
        g2.translate(-display.getWidth()/2, -display.getHeight()/2);


        //check translation
        g2.translate(movementX, movementY);
    }

    public void draw( Graphics g, boolean showEdges, boolean showDelaunay, boolean showSites, boolean showData )
    {
        this.zoom = display.zoom;
        this.movementX = display.movementX;
        this.movementY = display.movementY;
        this.g = g;
        g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        checkMovement();

        drawVoronoiArea();
        drawBounding();

        drawVoronoiCluster();
        drawClusterNodes(showSites, showData);

        drawDelaunay(showDelaunay);
        //drawBoundary();

        drawNodes();
        drawEdges( showEdges );
        drawTestEdge();
    }

    private void drawTestEdge()
    {
        for( TestEdge edge : t_edges )
        {
            edge.draw( g2, Color.BLACK );
        }
    }

    private void drawVoronoiCluster()
    {
        for( int i = 0; i < siteCluster.length; i++ ) {
            if (siteCluster[i] != null) {

                g2.setColor(Colors.circleFill);
                for (Site s : siteCluster[i]) {
                    PolygonSimple poly = s.getPolygon();
                    if (poly != null) {
                        g2.setColor(siteColours[i]);
                        g2.fill(poly);
                        g2.setColor(Color.BLACK);
                        g2.draw(poly);
                    }
                }

                g2.setColor(Color.GRAY);
                for (Site site : siteCluster[i]) {
                    double radius = 10;
                    Ellipse2D.Double shape = new Ellipse2D.Double(site.getPoint().getX() - (radius / 2), site.getPoint().getY() - (radius / 2), radius, radius);
                    g2.fill(shape);
                }
            }
        }
    }

    private void drawEdges( boolean showEdges )
    {
        if( showEdges ) {
            for( Edge edge : edges )
            {
                edge.draw( g2, Color.BLACK );
            }
        }
    }

    public void addTestEdge( ArrayList<TestEdge> t_edges )
    {
        this.t_edges = t_edges;
    }

    private void drawNodes()
    {
        for (Node node : nodes) {
            node.draw2(g2, Color.BLACK);
        }
    }

    private void drawBoundary()
    {
        if( newBorder != null ) {
            g2.setColor(Color.RED);
            g2.draw(newBorder);
        }
    }

    private void drawDelaunay( boolean showDelaunay )
    {
        if( showDelaunay ) {
            for (DelaunayEdge edge : d_edges) {
                edge.draw(g2, Color.RED);
            }
        }
    }

    public void setNormalNodes( ArrayList<Node> nodes )
    {
        this.nodes = nodes;
    }

    public void setNormalEdges( ArrayList<Edge> edges )
    {
        this.edges = edges;
    }

    private void drawClusterNodes(boolean showSites, boolean showData)
    {
        if( showSites) {
            g2.setColor(Color.GRAY);
            for (Site site : sites) {
                double radius = 10;
                Ellipse2D.Double shape = new Ellipse2D.Double(site.getPoint().getX() - (radius / 2), site.getPoint().getY() - (radius / 2), radius, radius);
                g2.fill(shape);
            }
        }

        if( showData ) {
            for (Cluster Cnode : clusternodes) {
                double radius = 10;
                Cnode.draw(g2, radius, Color.BLACK);
            }
        }
    }

    private void drawBounding()
    {
        if( boundingPolygon != null ) {
            g2.setColor(Color.BLACK);
            g2.draw(boundingPolygon);
        }
    }

    public void addBounding( PolygonSimple boundingPolygon )
    {
        this.boundingPolygon = boundingPolygon;
    }
    public void addBorderLines( PolygonSimple newBorder )
    {
        this.newBorder = newBorder;
    }

    public void addSites( OpenList sites )
    {
        this.sites = sites;
    }

    public void addSites2( OpenList sitesC, int index )
    {
        this.siteCluster[index] = sitesC;
    }

    public void addDelaunay( ArrayList<DelaunayEdge> d_edges )
    {
        this.d_edges = d_edges;
    }

    private void drawVoronoiArea()
    {
        double area = 0;
        double want = 0;
        double error = 0;
        double negerror = 0;
        g2.setColor(Colors.circleFill);
        for( Site s : sites )
        {
            PolygonSimple poly = s.getPolygon();
            if (poly != null) {
                area = poly.getArea();
                want = boundingPolygon.getArea()*s.getPercentage();
                error = area/want;
                error = (1-error);
                if( error < 0 )
                {
                    negerror = (error*-1);
                    error = 0;
                }
                error = (error*255);
                negerror = (negerror*255);
                if( error >= 255 )
                {
                    error = 255;
                }
                if( negerror >= 255 )
                {
                    negerror = 255;
                }
                Color co = new Color((255-(int)negerror), 255-((int)error+(int)negerror), 255-((int)error));
                g2.setColor(co);
                g2.fill(poly);
                g2.setColor(Color.GREEN);
                g2.draw(poly);
                negerror = 0;
                error = 0;
            }
        }
    }
}
