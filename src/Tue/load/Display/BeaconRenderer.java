package Tue.load.Display;

import Tue.load.Vector2;
import Tue.objects.Cluster;
import Tue.objects.Node;
import Tue.objects.VectorGraphic;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

/**
 * Created by s138362 on 23-6-2016.
 */
public class BeaconRenderer
{

    private int stepsize = 10;
    private double graphScaling = 0;
    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<VectorGraphic> gradientFields = new ArrayList<VectorGraphic>();
    private boolean once = true;

    private double[][] distortionGrid = new double[3000][3000];

    private int nodeRadius = 8;
    private int iterationTest = 0;

    private double divideScale = 50;
    private double dist = 0;

    public BeaconRenderer( ArrayList<Cluster> clusters, double graphScaling )
    {
        this.clusters = clusters;
        this.graphScaling = graphScaling;

    }

    public void drawBeaconGradient( Graphics2D g2, double[] nodeToCluster, Node node )
    {

        iterationTest++;
        if( once )
        {
            for( int i = 0; i < nodeToCluster.length; i++ )
            {
                dist += nodeToCluster[i];
            }
            dist = (dist/nodeToCluster.length);

            once = false;
            fillDistortionArray( nodeToCluster );
            fillGradientArray();
        }

        drawGrid( g2, nodeToCluster );
        drawRadii( g2, nodeToCluster );

        drawVectorField( g2 );
        node.drawNode(g2);

//        if( (iterationTest % 100) == 0 )
//        {
//            once = true;
//        }
    }

    private void fillGradientArray()
    {
        Vector2 pos;
        for( int i = 0; i < 1200; i+=stepsize ) {
            for( int j = 0; j < 800; j+=stepsize ) {
                pos = new Vector2(i+(stepsize/2), j+(stepsize/2));
                Vector2 expansionResult = getDerivativeExpansion( pos.getX(), pos.getY(), dist );
                Vector2 contractionResult = getDerivativeContraction( pos.getX(), pos.getY(), dist );

                //find which function is greater.
                if( Math.abs(expansionResult.getX()) > Math.abs(contractionResult.getX()) )
                {
                    contractionResult.x = expansionResult.getX();
                }

                if( Math.abs(expansionResult.getY()) > Math.abs(contractionResult.getY()) )
                {
                    contractionResult.y = expansionResult.getY();
                }

                //invert the result to get the proper gradient
                gradientFields.add( new VectorGraphic( pos, expansionResult ));
            }
        }
    }

    private void drawVectorField( Graphics2D g2 )
    {
        for( VectorGraphic v : gradientFields)
        {
            v.draw(g2);
        }
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

    private void fillDistortionArray( double[] nodeToCluster )
    {
        int index1 = 0;
        int index2 = 0;
        //we hard code the width and height in here because it is meant as a test not as a stable implementation
        for( int i = 0; i < 1200; i+=stepsize ) {
            for( int j = 0; j < 800; j+=stepsize ) {
                distortionGrid[index1][index2] = getDistortionNodeScale(i+(stepsize/2), j+(stepsize/2), nodeToCluster );
                index2++;
            }
            index2 = 0;
            index1++;
        }
        index2 = 0;
        index1 = 0;
    }

    private void drawGrid( Graphics2D g2, double[] nodeToCluster )
    {
        double distortion = 0;
        double highest = 0;
        double lowest = 99999;

        for( int i = 0; i < (int)(1200/stepsize); i++ )
        {
            for( int j = 0; j < (int)(800/stepsize); j++ )
            {
                if( highest < distortionGrid[i][j] )
                {
                    highest = distortionGrid[i][j];
                }
                if( lowest > distortionGrid[i][j] )
                {
                    lowest = distortionGrid[i][j];
                }
            }
        }

        for( int i = 0; i < (int)(1200/stepsize); i++ ) {
            for (int j = 0; j < (int)(800/stepsize); j ++ ) {
                distortion = distortionGrid[i][j];

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
                Rectangle2D.Double shape = new Rectangle2D.Double(i*stepsize, j*stepsize, stepsize+1, stepsize+1 );
                //Ellipse2D.Double shape = new Ellipse2D.Double(((i*stepsize) + (stepsize / 2)) - (nodeRadius / 2), ((j*stepsize) + (stepsize / 2)) - (nodeRadius / 2), nodeRadius, nodeRadius);
                g2.fill(shape);
            }
        }
    }

    private double getDistortionNodeScale( double xPos, double yPos, double[] nodeToCluster )
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
            contractionlocal = (((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]) / mapping[i]);
            expansionlocal = (mapping[i] / ((graphScaling/divideScale)*nodeToCluster[clusters.get(i).getNumber()]));

            if( contractionlocal >= expansionlocal )
            {
                distortion += contractionlocal;
            }
            else if( expansionlocal >= contractionlocal )
            {
                distortion += expansionlocal;
            }
        }
        distortion = (distortion/total);

        return distortion;
    }

    private double getDistortionNode(double xPos, double yPos, double[] nodeToCluster )
    {
        double distortion = 0;

        double contractionlocal = 0;
        double contractionmax = 0;
        double expansionlocal = 0;
        double expansionmax = 0;
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
            contractionlocal = ((graphScaling*nodeToCluster[clusters.get(i).getNumber()]) / mapping[i]);
            expansionlocal = (mapping[i] / (graphScaling*nodeToCluster[clusters.get(i).getNumber()]));

            if( contractionlocal >= contractionmax )
            {
                System.out.println("test2");
                contractionmax = contractionlocal;
            }
            else if( expansionlocal >= expansionmax )
            {
                System.out.println("test1");
                expansionmax = expansionlocal;
            }
        }
        distortion = contractionmax*expansionmax;

        return distortion;
    }

    private Vector2 getDerivativeExpansion( double xPos, double yPos, double dist)
    {
        double derivativeExpansionXTotal = 0;
        double derivativeExpansionYTotal = 0;

        for( Cluster c : clusters )
        {
            derivativeExpansionXTotal += getDerivativeExpansionX(xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
            derivativeExpansionYTotal += getDerivativeExpansionY(xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
        }

        derivativeExpansionXTotal = ( derivativeExpansionXTotal/clusters.size() );
        derivativeExpansionYTotal = ( derivativeExpansionYTotal/clusters.size() );

        return (new Vector2( derivativeExpansionXTotal, derivativeExpansionYTotal ));
    }

    private double getDerivativeExpansionX( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivate of expansion over x
        double result = (nodeX-clusterX)/(Math.sqrt(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2))*dist);

        return result;
    }

    private double getDerivativeExpansionY( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of expansion over y
        double result = (nodeY-clusterY)/(Math.sqrt(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2))*dist);

        return result;
    }

    private Vector2 getDerivativeContraction( double xPos, double yPos, double dist )
    {
        double derivativeContractionXTotal = 0;
        double derivativeContractionYTotal = 0;

        for( Cluster c : clusters )
        {
            derivativeContractionXTotal += getDerivativeContractionX( xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
            derivativeContractionYTotal += getDerivativeContractionY( xPos, yPos, c.getPos().getX(), c.getPos().getY(), dist);
        }

        derivativeContractionXTotal = ( derivativeContractionXTotal/clusters.size() );
        derivativeContractionYTotal = ( derivativeContractionYTotal/clusters.size() );

        return (new Vector2( derivativeContractionXTotal, derivativeContractionYTotal ));
    }

    private double getDerivativeContractionX( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of contraction over x
        double result = (nodeX-clusterX)*dist/(Math.pow(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2), (3/2)));

        return result;
    }

    private double getDerivativeContractionY( double nodeX, double nodeY, double clusterX, double clusterY, double dist )
    {
        //derivative of contraction over y
        double result = (nodeY-clusterY)*dist/(Math.pow(Math.pow((nodeX-clusterX), 2) + Math.pow((nodeY-clusterY), 2), (3/2)));

        return result;
    }
}
