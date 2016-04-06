package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Renderer
{
    private ArrayList<Cluster> clusternodes = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();
    private ArrayList<DelaunayEdge> d_edges =  new ArrayList<DelaunayEdge>();
    private OpenList sites;

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;
    private PolygonSimple newBorder = null;

    public Renderer(ArrayList<Cluster> clusternodes, ArrayList<ClusterEdge> clusteredges )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;

        sites = new OpenList();
    }

    public void draw( Graphics g, boolean showEdges )
    {
        this.g = g;
        g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawVoronoiArea();
        drawNodes();
        drawEdges( showEdges );
        drawBounding();
        drawDelaunay();
        //drawBoundary();
    }

    private void drawBoundary()
    {
        if( newBorder != null ) {
            g2.setColor(Color.RED);
            g2.draw(newBorder);
        }
    }

    private void drawDelaunay()
    {
        g2.setColor(Color.YELLOW);
        for( DelaunayEdge edge : d_edges )
        {
            edge.draw( g2, Color.YELLOW );
        }
    }

    private void drawNodes()
    {
        g2.setColor(Color.BLACK);
        for (Site site : sites )
        {
            double radius = 10;
            Ellipse2D.Double shape = new Ellipse2D.Double(site.getPoint().getX()-(radius/2), site.getPoint().getY()-(radius/2), radius, radius);
            g2.fill(shape);
        }

        for (Cluster Cnode : clusternodes)
        {
            double radius = 10;
            Cnode.draw(g2, radius, Color.RED);
        }
    }

    private void drawEdges( boolean showEdges )
    {
        for (ClusterEdge edge : clusteredges )
        {
            if( showEdges ) {
                edge.draw(g2, Color.BLACK);
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
        g2.setColor(Colors.circleFill);
        for( Site s : sites )
        {
            PolygonSimple poly = s.getPolygon();
            if (poly != null) {
                g2.setColor(Color.WHITE);
                g2.fill(poly);
                g2.setColor(Color.GREEN);
                g2.draw(poly);
            }
        }
    }
}
