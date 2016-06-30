package Tue.load.Display;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.debuge.Colors;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.Cluster;
import Tue.objects.Node;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 22-6-2016.
 */
public class RenderVoronoi
{
    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    private Color[] siteColours;
    private Random rand;

    public RenderVoronoi( ArrayList<Cluster> clusters)
    {
        rand = new Random(5581);

        this.clusters = clusters;

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
        siteColours = new Color[clusters.size()];
        for( int i = 0; i < clusters.size(); i++ )
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

    public void drawVoronoiCluster(Graphics2D g2, OpenList[] siteCluster )
    {
        //the bounding lines of the voronoi
        for( int i = 0; i < siteCluster.length; i++ ) {
            if (siteCluster[i] != null) {

                g2.setColor(Colors.circleFill);
                for (Site s : siteCluster[i]) {
                    PolygonSimple poly = s.getPolygon();
                    if (poly != null) {
                        double distanceToZeroX = poly.getCentroid().getX();
                        double distanceToZeroY = poly.getCentroid().getY();
                        double edge = 1;
                        int transparent = 0;

                        g2.setColor( Color.BLACK );
                        g2.fill(poly);
                        g2.setColor( new Color( 0, 0, 0, 120) );
                        g2.draw(poly);

                        PolygonSimple[] p = new PolygonSimple[4];
                        for( int j = 0; j < 2; j++ ) {
                            p[j] = new PolygonSimple();
                            g2.setColor(new Color(siteColours[i].getRed(), siteColours[i].getGreen(), siteColours[i].getBlue(), 200 - transparent));

                            for (int k = 0; k < poly.getNumPoints(); k++) {
                                p[j].add(poly.getXPoints()[k], poly.getYPoints()[k]);
                            }

                            p[j].translate(-distanceToZeroX, -distanceToZeroY);
                            p[j].scale(edge);
                            p[j].translate(distanceToZeroX, distanceToZeroY);

                            g2.fill(p[j]);

                            transparent = (transparent + 20);
                            edge = (edge - 0.07);
                        }
                    }
                }
            }
        }
    }


    public void drawClusterNodes( Graphics2D g2, OpenList sites, boolean showSites, boolean showData )
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
            for (Cluster c : clusters) {
                double radius = 10;
                c.draw(g2, radius, Color.BLACK);
                c.drawText(g2);
            }
        }
    }

    public void drawBounding( Graphics2D g2, PolygonSimple boundingPolygon )
    {
        if( boundingPolygon != null ) {
            g2.setColor(Color.BLACK);
            g2.draw(boundingPolygon);
        }
    }

    public void drawNodes( Graphics2D g2, ArrayList<Node> nodes )
    {
        for( Node n : nodes )
        {
            n.drawNode( g2 );
        }
    }

    public void drawVoronoiArea( Graphics2D g2, boolean labelfill, OpenList sites, OpenList[] siteCluster, PolygonSimple boundingPolygon )
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
            //filling the voronoi
            for( int i = 0; i < siteCluster.length; i++ ) {
                if (siteCluster[i] != null) {

                    g2.setColor(Colors.circleFill);
                    PolygonSimple poly = clusters.get(i).getSite().getPolygon();
                    if (poly != null) {
                        g2.setColor( siteColours[i] );
                        g2.fill(poly);
                        g2.setColor( new Color( 0, 0, 0, 120) );
                        g2.draw(poly);
                    }
                }
            }
        }
    }
}
