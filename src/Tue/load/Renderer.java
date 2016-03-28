package Tue.load;

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

    public Renderer( ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
    }

    public void draw( Graphics g, boolean showEdges )
    {
        drawNodes(g, clusternodes );
        drawEdges(g, clusteredges, showEdges );
    }

    private void drawNodes( Graphics g, ArrayList<ClusterNode> clusternodes )
    {
        for (ClusterNode Cnode : clusternodes)
        {
            int radius = 20;
            g.fillOval((int)(Cnode.getPos().x-(radius/2)), (int)(Cnode.getPos().y-(radius/2)), radius, radius);
        }
    }

    private void drawEdges( Graphics g, ArrayList<ClusterEdge> clusteredges, boolean showEdges )
    {
        for (ClusterEdge edge : clusteredges )
        {
            if( showEdges ) {
                g.drawLine((int) edge.getSource().getPos().x, (int) edge.getSource().getPos().y, (int) edge.getDest().getPos().x, (int) edge.getDest().getPos().y);
            }
        }
    }

}
