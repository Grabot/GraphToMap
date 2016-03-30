package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;
import Tue.objects.VoronoiEdge;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.HashSet;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Renderer
{
    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();
    private OpenList sites;

    private HashSet<VoronoiEdge> voredges = new HashSet<VoronoiEdge>();

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;

    public Renderer( ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges )
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

        drawBounding();
        drawVoronoiArea();
        drawNodes();
        drawEdges( showEdges );
    }

    private void drawNodes()
    {
        g2.setColor(Color.BLUE);
        for (Site site : sites )
        {
            int radius = 10;
            Ellipse2D.Double shape = new Ellipse2D.Double(site.getPoint().getX()-(radius/2), site.getPoint().getY()-(radius/2), radius, radius);
            g2.fill(shape);
            //g.fillOval((int)(Cnode.getPos().x-(radius/2)), (int)(Cnode.getPos().y-(radius/2)), radius, radius);
        }

//        g2.setColor(Color.BLUE);
//        for (ClusterNode Cnode : clusternodes)
//        {
//            int radius = 10;
//            Ellipse2D.Double shape = new Ellipse2D.Double(Cnode.getPos().x-(radius/2), Cnode.getPos().y-(radius/2), radius, radius);
//            g2.fill(shape);
//            //g.fillOval((int)(Cnode.getPos().x-(radius/2)), (int)(Cnode.getPos().y-(radius/2)), radius, radius);
//        }
    }

    private void drawEdges( boolean showEdges )
    {
        g2.setColor(Color.black);
        for (ClusterEdge edge : clusteredges )
        {
            if( showEdges ) {
                Shape shape = new Line2D.Double(edge.getSource().getPos().x, edge.getSource().getPos().y, edge.getDest().getPos().x, edge.getDest().getPos().y);
                g2.draw(shape);
            }
        }
    }

    private void drawBounding()
    {
        if( boundingPolygon != null )
        {
            double[] xPoints = boundingPolygon.getXPoints();
            double[] yPoints = boundingPolygon.getYPoints();

            for( int i = 0; i < boundingPolygon.length; i++ )
            {
                if( i != (boundingPolygon.length-1))
                {
                    g.drawLine((int)xPoints[i], (int)yPoints[i], (int)xPoints[i+1], (int)yPoints[i+1]);
                }
                else
                {
                    g.drawLine((int)xPoints[i], (int)yPoints[i], (int)xPoints[0], (int)yPoints[0] );
                }
            }
        }
    }

    public void addBounding( PolygonSimple boundingPolygon )
    {
        this.boundingPolygon = boundingPolygon;
    }

    public void addVoronoiEdges( HashSet<VoronoiEdge> voredges )
    {
        this.voredges = voredges;
    }

    public void addSites( OpenList sites )
    {
        this.sites = sites;
    }

    private void drawVoronoiArea()
    {
//        for( VoronoiEdge edge : voredges ) {
//            Shape shape = new Line2D.Double(edge.getSource().x, edge.getSource().y, edge.getDest().x, edge.getDest().y);
//            g2.draw(shape);
//        }

        g2.setColor(Colors.circleFill);
        for( Site s : sites )
        {
            PolygonSimple poly = s.getPolygon();
            if (poly != null) {
                g2.setColor(Color.CYAN);
                g2.fill(poly);
                g2.setColor(Color.red);
                g2.draw(poly);
            }
        }
    }

}
