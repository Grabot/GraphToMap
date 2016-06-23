package Tue.load.Display;

import Tue.load.Vector2;
import Tue.objects.Cluster;
import Tue.objects.Node;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;

/**
 * Created by s138362 on 23-6-2016.
 */
public class BeaconRenderer
{

    private int stepsize = 4;
    private double graphScaling = 0;
    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    public BeaconRenderer( ArrayList<Cluster> clusters, double graphScaling )
    {
        this.clusters = clusters;
        this.graphScaling = graphScaling;
    }

    public void drawBeaconGradient( Graphics2D g2, double[] nodeToCluster, Node node )
    {
        drawGrid( g2, nodeToCluster );
        drawRadii( g2, nodeToCluster );

        node.drawNode(g2);

    }

    private void drawRadii(  Graphics2D g2, double[] nodeToCluster )
    {
        for( int i = 0; i < nodeToCluster.length; i++ )
        {
            double radius = (nodeToCluster[i]*50);
            Color c1 = new Color(27, 0, 255, 200 );
            g2.setColor(c1);
            Ellipse2D.Double shape = new Ellipse2D.Double(clusters.get(i).getPos().getX()-(radius/2), clusters.get(i).getPos().getY()-(radius/2), radius, radius);
            g2.draw(shape);
        }
    }

    private void drawGrid( Graphics2D g2, double[] nodeToCluster )
    {
        double distortion = 0;
        double highest = 0;
        double lowest = 99999;


        //we hard code the width and height in here because it is meant as a test not as a stable implementation
        for( int i = 0; i < 1200; i+=stepsize ) {
            for( int j = 0; j < 800; j+=stepsize ) {
                distortion = getDistortionNode(i+(stepsize/2), j+(stepsize/2), nodeToCluster );
                if( highest < distortion )
                {
                    highest = distortion;
                }
                if( lowest > distortion )
                {
                    lowest = distortion;
                }

            }
        }

        for( int i = 0; i < 1200; i+=stepsize ) {
            for (int j = 0; j < 800; j += stepsize) {
                distortion = getDistortionNode(i + (stepsize / 2), j + (stepsize / 2), nodeToCluster);

                distortion = (distortion-lowest);
                distortion = (distortion * ((255/(highest-lowest))));

                if( distortion > 255 )
                {
                    distortion = 255;
                }
                else if( distortion < 0 )
                {
                    distortion = 0;
                }

                g2.setColor(new Color(((int)(distortion)), 255-(int)(distortion), 0));
                if( distortion == 0 )
                {
                    g2.setColor( Color.BLACK );
                }
                Ellipse2D.Double shape = new Ellipse2D.Double((i + (stepsize / 2)) - (6 / 2), (j + (stepsize / 2)) - (6 / 2), 6, 6);
                g2.fill(shape);
            }
        }

//        c1 = new Color(0, 0, 0);
//        g2.setColor(c1);
//        for( int i = 0; i < 1201; i+=stepsize ) {
//            Line2D.Double line = new Line2D.Double(i, 0, i, 800 );
//            g2.draw(line);
//        }
//
//        for( int i = 0; i < 801; i+=stepsize )
//        {
//            Line2D.Double line = new Line2D.Double(0, i, 1200, i );
//            g2.draw(line);
//        }
    }

    private double getDistortionNode(double xPos, double yPos, double[] nodeToCluster )
    {
        double distortion = 0;

        double contractionlocal = 0;
        double expansionlocal = 0;
        int total = 0;

        double[] mapping = new double[clusters.size()];

        for (int i = 0; i < clusters.size(); i++) {
            Vector2 node1 = new Vector2(xPos, yPos);
            for (int j = 0; j < clusters.size(); j++)
            {
                Vector2 node2 = clusters.get(j).getPos();
                //get the actual distances between all nodes to calculate the distorion metrics
                mapping[j] = node1.distance(node2);
            }
        }

        for( int i = 0; i < mapping.length; i++ )
        {
            total++;
            contractionlocal = (((graphScaling/100)*nodeToCluster[clusters.get(i).getNumber()]) / mapping[i]);
            expansionlocal = (mapping[i] / ((graphScaling/100)*nodeToCluster[clusters.get(i).getNumber()]));

            if( contractionlocal >= expansionlocal )
            {
                distortion = (distortion + contractionlocal);
            }
            else
            {
                distortion = (distortion + expansionlocal);
            }
        }
        distortion = (distortion/total);

        return distortion;
    }
}
