package Tue.load.Display;

import Tue.load.LabelObject;
import Tue.load.Vector2;
import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.*;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
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

    private Color[] siteColours;

    private Graphics g;
    private Graphics2D g2;
    private PolygonSimple boundingPolygon = null;
    private PolygonSimple newBorder = null;
    private Random rand;

    private Display display;
    private LabelObject[] labels;
    private boolean labelfill = false;

    private double[] nodeToCluster;

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
        Color[] brewer = new Color[12];
        brewer[0] = new Color(166,206,227);
        brewer[1] = new Color(31,120,180);
        brewer[2] = new Color(178,223,138);
        brewer[3] = new Color(51,160,44);
        brewer[4] = new Color(251,154,153);
        brewer[5] = new Color(227,26,28);
        brewer[6] = new Color(253,191,111);
        brewer[7] = new Color(255,127,0);
        brewer[8] = new Color(202,178,214);
        brewer[9] = new Color(106,61,154);
        brewer[10] = new Color(255,255,153);
        brewer[11] = new Color(177,89,40);
        siteColours = new Color[clusternodes.size()];
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            if( i > 11 ) {
                siteColours[i] = new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
            }
            else
            {
                siteColours[i] = brewer[i];
            }
        }

    }

    private String readFile(String filename) throws IOException {
        String content = null;
        File file = new File(filename); //for ex foo.txt
        FileReader reader = null;
        try {
            reader = new FileReader(file);
            char[] chars = new char[(int) file.length()];
            reader.read(chars);
            content = new String(chars);
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if(reader !=null){reader.close();}
        }
        return content;
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

        drawVoronoiArea();
        drawBounding();

        drawVoronoiCluster();
        drawClusterNodes(showSites, showData);

        drawDelaunay(showDelaunay);
        //drawBoundary();

        drawLabels();
        drawEdges( showEdges );
        //drawNodes();
        //drawCircleTest();
        //drawRadiusTest();
        //drawSpecialPoint();
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
                        g2.setColor( new Color( 0, 0, 0, 120) );
                        g2.draw(poly);
                    }
                }
            }
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

    private void drawNodes()
    {
        for( Node n : nodes )
        {
            n.drawNode( g2 );
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
                Cnode.drawText(g2);
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
        if( !labelfill ) {
            double area = 0;
            double want = 0;
            double error = 0;
            double negerror = 0;
            g2.setColor(Colors.circleFill);
            for (Site s : sites) {
                PolygonSimple poly = s.getPolygon();
                if (poly != null) {
                    area = poly.getArea();
                    want = boundingPolygon.getArea() * s.getPercentage();
                    error = area / want;
                    error = (1 - error);
                    if (error < 0) {
                        negerror = (error * -1);
                        error = 0;
                    }
                    error = (error * 255);
                    negerror = (negerror * 255);
                    if (error >= 255) {
                        error = 255;
                    }
                    if (negerror >= 255) {
                        negerror = 255;
                    }
                    Color co = new Color((255 - (int) negerror), 255 - ((int) error + (int) negerror), 255 - ((int) error));
                    g2.setColor(co);
                    g2.fill(poly);
                    g2.setColor(Color.GREEN);
                    g2.draw(poly);
                    negerror = 0;
                    error = 0;
                }
            }
        }
        else
        {
            for( int i = 0; i < siteCluster.length; i++ ) {
                if (siteCluster[i] != null) {

                    g2.setColor(Colors.circleFill);
                    PolygonSimple poly = clusternodes.get(i).getSite().getPolygon();
                    if (poly != null) {
                        g2.setColor(siteColours[i]);
                        g2.fill(poly);
                        g2.setColor( new Color( 0, 0, 0, 120) );
                        g2.draw(poly);
                    }
                }
            }
        }
    }

    public void addNodeToClusterTest( double[] nodeToCluster )
    {
        this.nodeToCluster = nodeToCluster;
    }

    private void drawRadiusTest()
    {
        if( nodeToCluster != null ) {
            for (Cluster c : clusternodes) {
                Color color = new Color(0, 0, 0);
                g2.setColor(color);
                //I think the draw shape commands uses diameter instead of radius, cuase multiplying by 2 works well.
                double radius = (nodeToCluster[c.getNumber()]*2);

                Ellipse2D.Double shape = new Ellipse2D.Double(c.getPos().getX() - (radius / 2), c.getPos().getY() - (radius / 2), radius, radius);
                g2.draw(shape);
            }
        }
    }

    private void drawCircleTest()
    {
        g2.setColor( new Color(247, 0, 202));

        for( Vector2 v : circleIntersections ) {
            double radius = 10;
            Ellipse2D.Double shape = new Ellipse2D.Double(v.getX() - (radius / 2), v.getY() - (radius / 2), radius, radius);
            g2.fill(shape);
        }
    }

    private ArrayList<Vector2> circleIntersections = new ArrayList<Vector2>();

    public void addCircleTest( ArrayList<Vector2> circleIntersections )
    {
        this.circleIntersections = circleIntersections;
    }

    private Vector2 point = new Vector2(0, 0);
    public void addSpecialPoint( Vector2 point )
    {
        this.point = point;
    }

    private void drawSpecialPoint()
    {
        g2.setColor( new Color(247, 0, 9));

        double radius = 10;
        Ellipse2D.Double shape = new Ellipse2D.Double(point.getX() - (radius / 2), point.getY() - (radius / 2), radius, radius);
        g2.fill(shape);
    }
}
