package Tue.load;

import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;

import java.awt.*;
import java.util.ArrayList;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Renderer
{
    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();

    private Graphics g;

    private PolygonSimple boundingPolygon = null;

    public Renderer( ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
    }

    public void draw( Graphics g, boolean showEdges )
    {
        this.g = g;
        drawNodes( );
        drawEdges( showEdges );
        //drawBounding();
        drawVoronoiArea();
    }

    private void drawNodes()
    {
        for (ClusterNode Cnode : clusternodes)
        {
            int radius = 20;
            g.fillOval((int)(Cnode.getPos().x-(radius/2)), (int)(Cnode.getPos().y-(radius/2)), radius, radius);
        }
    }

    private void drawEdges( boolean showEdges )
    {
        for (ClusterEdge edge : clusteredges )
        {
            if( showEdges ) {
                g.drawLine((int) edge.getSource().getPos().x, (int) edge.getSource().getPos().y, (int) edge.getDest().getPos().x, (int) edge.getDest().getPos().y);
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

    public void clearVoronoi()
    {
        polys.clear();
    }

    public void addVoronoiArea( PolygonSimple area )
    {
        polys.add( area );
    }

    private void drawVoronoiArea()
    {
        for( PolygonSimple pol : polys )
        {
            double[] xPoints = pol.getXPoints();
            double[] yPoints = pol.getYPoints();

            for( int i = 0; i < xPoints.length; i++ )
            {
                if( xPoints[i] != 0 )
                {
                    if( i != (pol.length-1))
                    {
                        g.drawLine((int)xPoints[i], (int)yPoints[i], (int)xPoints[i+1], (int)yPoints[i+1] );
                    }
                    else
                    {
                        g.drawLine((int)xPoints[i], (int)yPoints[i], (int)xPoints[0], (int)yPoints[0] );
                    }
                }
            }
        }
    }

}
