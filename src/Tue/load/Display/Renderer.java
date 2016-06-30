package Tue.load.Display;

import Tue.load.LabelObject;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.awt.*;
import java.awt.geom.*;
import java.io.*;
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
    private ArrayList<PolygonSimple> nodeLabelRects = new ArrayList<PolygonSimple>();
    private ArrayList<DelaunayEdge> d_edges =  new ArrayList<DelaunayEdge>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private ArrayList<TestEdge> t_edges = new ArrayList<TestEdge>();
    private OpenList sites;
    private OpenList[] siteCluster;

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;
    private PolygonSimple newBorder = null;

    private Display display;
    private LabelObject[] labels;
    private boolean labelfill = false;
    private boolean beaconPosTest = false;

    private double[] nodeToCluster;

    private RenderVoronoi vorRender;
    private BeaconRenderer beaconRender;
    private Node Beaconnode;
    private Cluster BeaconCluster;

    public Renderer(Display display, ArrayList<Cluster> clusternodes, ArrayList<ClusterEdge> clusteredges, double graphScaling )
    {
        this.display = display;
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;

        sites = new OpenList();
        siteCluster = new OpenList[clusternodes.size()];
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            siteCluster[i] = new OpenList();
        }

        vorRender = new RenderVoronoi( clusternodes );
        beaconRender = new BeaconRenderer( clusternodes, graphScaling );

    }

    private void checkMovement()
    {
        double zoom = display.zoom;
        double movementX = display.movementX;
        double movementY = display.movementY;

        //check zooming
        g2.translate(display.getWidth()/2, display.getHeight()/2);
        g2.scale(zoom, zoom);
        g2.translate(-display.getWidth()/2, -display.getHeight()/2);

        //check translation
        g2.translate(movementX, movementY);
    }

    public void draw( Graphics g, boolean showEdges, boolean showDelaunay, boolean showSites, boolean showData )
    {
        this.g = g;
        g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        checkMovement();

        vorRender.drawVoronoiArea( g2, labelfill, sites, siteCluster, boundingPolygon );
        vorRender.drawBounding( g2, boundingPolygon );

        vorRender.drawVoronoiCluster( g2, siteCluster );
        vorRender.drawClusterNodes( g2, sites, showSites, showData );

        drawDelaunay(showDelaunay);
        //drawBoundary();

        drawLabels();
        drawEdges( showEdges );
        //vorRender.drawNodes( g2, nodes );

        if( beaconPosTest ) {
            //beaconRender.drawBeaconGradient(g2, nodeToCluster, BeaconCluster, Beaconnode);
        }


        g2.setColor( Color.BLACK );
        PolygonSimple initial = new PolygonSimple();
        initial.add(0, 100);
        initial.add(200, 0);
        initial.add(400, 100);
        initial.add(400, 300);
        initial.add(200, 400);
        initial.add(0, 300);
        g2.fill(initial);

        double edge = 1;
        int transparent = 0;
        double distanceToZeroX = initial.getCentroid().x;
        double distanceToZeroY = initial.getCentroid().y;
        PolygonSimple[] p = new PolygonSimple[4];
        for( int i = 0; i < 4; i++ ) {
            p[i] = new PolygonSimple();
            g2.setColor(new Color(166, 206, 227, (150 - transparent)));

            p[i] = initial;
            p[i].translate( -distanceToZeroX, -distanceToZeroY );
            p[i].scale(edge);
            p[i].translate( distanceToZeroX, distanceToZeroY );

            g2.fill(p[i]);

            transparent = (transparent+20);
            edge = (edge-0.1);
        }
    }

    private void drawEdges( boolean showEdges )
    {
        if( showEdges ) {
            for( Edge edge : edges )
            {
                edge.draw( g2, new Color( 0, 0, 0, 140) );
            }
        }
    }

    public void addTestEdge( ArrayList<TestEdge> t_edges )
    {
        this.t_edges = t_edges;
    }

    private void drawLabels()
    {
        if( labelfill ) {
            for( int i = 0; i < labels.length; i++ )
            {
                PolygonSimple rect = labels[i].getNode().setRect(g2, display.zoominverse);
                labels[i].setRect(rect);
            }

            for( int i = (labels.length-1); i > -1; i-- )
            {
                PolygonSimple rect = labels[i].getRect();
                //if no rectangles overlap this one you can draw it
                //if a rectangle does overlap it, it means it's a better one, so don't draw this one
                if( !checkOverlapRects( rect ) || (display.zoom > 2))
                {
                    g2.setColor(new Color(0, 0, 0));
                    //we can draw the rectangle and the text (or just the text)
                    //g2.draw(rect);
                    labels[i].getNode().drawText(g2);
                    //these are saved for future checks for overlapping
                    nodeLabelRects.add(rect);
                }
            }
            //we drew all the labels, so clear the list for the next iteration
            nodeLabelRects.clear();
        }
    }


    private boolean checkOverlapRects( PolygonSimple rect )
    {
        for( PolygonSimple p : nodeLabelRects )
        {
            if( checkOverLap( rect, p ))
            {
                return true;
            }
        }
        return false;
    }

    private boolean checkOverLap( PolygonSimple rectSource, PolygonSimple rectCheck )
    {
        for( int i = 0; i < 4; i++ ) {
            if (pnpoly(4, rectSource.getXPoints(), rectSource.getYPoints(), rectCheck.getXPoints()[i], rectCheck.getYPoints()[i])) {
                return true;
            }
        }

        for( int i = 0; i < 4; i++ ) {
            if (pnpoly(4, rectCheck.getXPoints(), rectCheck.getYPoints(), rectSource.getXPoints()[i], rectSource.getYPoints()[i])) {
                return true;
            }
        }
        return false;
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

        labels = new LabelObject[nodes.size()];
        for( int i = 0; i < nodes.size(); i++ )
        {
            labels[i] = new LabelObject(nodes.get(i), null, nodes.get(i).getWeight() );
        }
        sortLabels();
        labelfill = true;
    }

    public void setBeaconBasedTest( double[] nodeToCluster, Cluster cl, Node node )
    {
        beaconPosTest = true;
        this.nodeToCluster = nodeToCluster;
        this.Beaconnode = node;
        this.BeaconCluster = cl;
    }

    private void sortLabels()
    {
        int n = labels.length;
        LabelObject temp = null;

        for(int i=0; i < n; i++){
            for(int j=1; j < (n-i); j++){

                if(labels[j-1].getNodeValue() > labels[j].getNodeValue()){
                    //swap the elements!
                    temp = labels[j-1];
                    labels[j-1] = labels[j];
                    labels[j] = temp;
                }

            }
        }
    }

    public void setNormalEdges( ArrayList<Edge> edges )
    {
        this.edges = edges;
    }

    public void addBounding( PolygonSimple boundingPolygon )
    {
        this.boundingPolygon = boundingPolygon;
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

}
