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
    private OpenList sites;

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;
    private PolygonSimple newBorder = null;
    private Random rand;

    public Renderer(ArrayList<Cluster> clusternodes, ArrayList<ClusterEdge> clusteredges)
    {
        rand = new Random();
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;

        sites = new OpenList();
    }

    public void draw( Graphics g, boolean showEdges, boolean showDelaunay, boolean showSites, boolean showData )
    {
        this.g = g;
        g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawVoronoiArea();
        drawClusterNodes(showSites, showData);
        drawBounding();
        drawDelaunay(showDelaunay);
        //drawBoundary();

        drawNodes();
        drawEdges( showEdges );

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

    private void drawNodes()
    {
        for (Node node : nodes) {
            node.draw(g2, 10, node.getColor());
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
            g2.setColor(Color.YELLOW);
            for (DelaunayEdge edge : d_edges) {
                edge.draw(g2, Color.YELLOW);
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
