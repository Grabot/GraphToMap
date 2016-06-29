package Tue.load.Display;

import Tue.load.Vector2;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.Cluster;
import Tue.objects.Node;
import Tue.objects.PolygonEdge;
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

    public void drawBeaconGradient( Graphics2D g2, double[] nodeToCluster, Cluster cl, Node node )
    {

        iterationTest++;
        if( once )
        {
            getDistortionNodeScale( node.getX(), node.getY(), nodeToCluster );
            for( int i = 0; i < nodeToCluster.length; i++ )
            {
                dist += nodeToCluster[i];
            }
            dist = (dist/nodeToCluster.length);

            once = false;
            fillDistortionArray( nodeToCluster );
            //fillGradientArray();
            fillGradientArrayCluster( cl );
        }

        drawGrid( g2, nodeToCluster );
        drawRadii( g2, nodeToCluster );

        drawVectorField( g2, cl );
        node.drawNode(g2);

//        if( (iterationTest % 100) == 0 )
//        {
//            once = true;
//        }
    }

    private void fillGradientArrayCluster( Cluster cl )
    {
        ArrayList<PolygonEdge> polygonEdges = new ArrayList<PolygonEdge>();
        Vector2 pos;
        PolygonSimple poly = cl.getSite().getPolygon();
        double[] xPoints = poly.getXPoints();
        double[] yPoints = poly.getYPoints();
        //just do 1 line for now
        double x2 = xPoints[2];
        double y2 = yPoints[2];
        double x3 = xPoints[3];
        double y3 = yPoints[3];

        for( int i = 0; i < poly.getNumPoints(); i++ ) {
            Vector2 from;
            Vector2 to;
            if ((i == (poly.getNumPoints() - 1))) {
                from = new Vector2(xPoints[i], yPoints[i]);
                to = new Vector2(xPoints[0], yPoints[0]);
            } else {
                from = new Vector2(xPoints[i], yPoints[i]);
                to = new Vector2(xPoints[i + 1], yPoints[i + 1]);
            }
            polygonEdges.add(new PolygonEdge(from, to));
        }
        PolygonEdge edge = new PolygonEdge( new Vector2(x2, y2), new Vector2(x3, y3) );

        for( int i = 0; i < 1200; i+=stepsize ) {
            for( int j = 0; j < 800; j+=stepsize ) {
                pos = new Vector2(i+(stepsize/2), j+(stepsize/2));

                if( pnpoly(poly.getNumPoints(), xPoints, yPoints, pos.getX(), pos.getY() ))
                {
                    Vector2 gradientCluster = new Vector2( 0, 0 );
                    for( PolygonEdge e : polygonEdges )
                    {
                        Vector2 gradientEdge = getDerivativePolyEdge( pos.getX(), pos.getY(), e.getFrom().getX(), e.getFrom().getY(), e.getTo().getX(), e.getTo().getY(), 100 );
                        if( Math.abs(gradientEdge.getX()) > Math.abs(gradientCluster.getX()) )
                        {
                            gradientCluster.x = gradientEdge.getX();
                            System.out.println("x: " + Math.abs(gradientEdge.getX()) );
                        }
                        if( Math.abs(gradientEdge.getY()) > Math.abs(gradientCluster.getY()) )
                        {
                            gradientCluster.y = gradientEdge.getY();
                            System.out.println("y: " + Math.abs(gradientEdge.getY()) );
                        }
                    }
                    if( gradientCluster.getX() > 1 )
                    {
                        gradientCluster.x = 1;
                    }
                    else if( gradientCluster.getX() < -1 )
                    {
                        gradientCluster.x = -1;
                    }

                    if( gradientCluster.getY() > 1 )
                    {
                        gradientCluster.y = 1;
                    }
                    else if( gradientCluster.getY() < -1 )
                    {
                        gradientCluster.y = -1;
                    }
                    gradientFields.add( new VectorGraphic( pos, gradientCluster ));
                }
            }
        }
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

                gradientFields.add( new VectorGraphic( pos, expansionResult ));
            }
        }
    }

    private void drawVectorField( Graphics2D g2, Cluster cl )
    {
        for( VectorGraphic v : gradientFields)
        {
            v.draw(g2);
        }

        PolygonSimple poly = cl.getSite().getPolygon();
        double[] xPoints = poly.getXPoints();
        double[] yPoints = poly.getYPoints();
        //just do 1 line for now
        double x2 = xPoints[2];
        double y2 = yPoints[2];
        double x3 = xPoints[3];
        double y3 = yPoints[3];

        g2.setColor(new Color(0, 36, 234));
        Line2D.Double line = new Line2D.Double(x2, y2, x3, y3);
        g2.draw(line);
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
                    //g2.setColor( Color.BLACK );
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



    private Vector2 getDerivativePolyEdge( double x1, double y1, double x2, double y2, double x3, double y3, double dist )
    {
        double x = getDerivativePolyEdgeX( x1, y1, x2, y2, x3, y3, dist );
        double y = getDerivativePolyEdgeY( x1, y1, x2, y2, x3, y3, dist );
        Vector2 edgeGradient = new Vector2(x, y);
        return edgeGradient;
    }

    private double getDerivativePolyEdgeX( double x1, double y1, double x2, double y2, double x3, double y3, double c )
    {
        double numerator = c*((y2-y3)*Math.sqrt(Math.pow((x3 - x2), 2) + Math.pow((y3 - y2), 2)));
        double denominator = Math.pow(((x3*(y1-y2))+(x1*(y2-y3))+(x2*(y3-y1))),2);

        return ((numerator/denominator)*-1);
    }

    private double getDerivativePolyEdgeY( double x1, double y1, double x2, double y2, double x3, double y3, double c )
    {
        double numerator = c*((x3-x2)*Math.sqrt(Math.pow((x3 - x2), 2) + Math.pow((y3 - y2), 2)));
        double denominator = Math.pow(((x3*(y1-y2))+(x1*(y2-y3))+(x2*(y3-y1))),2);

        return ((numerator/denominator)*-1);
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
}
