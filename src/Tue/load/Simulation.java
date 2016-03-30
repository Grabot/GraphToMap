package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.diagram.PowerDiagram;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;
import Tue.objects.VoronoiEdge;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Simulation
{

    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();

    private HashSet<VoronoiEdge> voredges = new HashSet<VoronoiEdge>();

    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private VoronoiCore core;

    private Renderer render;

    private double distanceBorder = 0;

    private int iterations = 0;

    public Simulation(Renderer render, ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges, int width, int height )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
        this.render = render;

        // normal list based on an array
        sites = new OpenList();
        core = new VoronoiCore();

        //create bounding polygon (the size of the panel)
        boundingPolygon = new PolygonSimple();
        boundingPolygon.add(0, 0);
        boundingPolygon.add(width, 0);
        boundingPolygon.add(width, height);
        boundingPolygon.add(0, height);
        render.addBounding(boundingPolygon);

        int amount=clusternodes.size();

        Random rand=new Random(200);

        for (int i=0;i<amount;i++){
            Site site = new Site(clusternodes.get(i).getPos().x, clusternodes.get(i).getPos().y);
            //site.setPercentage(rand.nextFloat());
            site.setWeight(100);
            sites.add(site);
        }
        sites.get(0).setPercentage(2);
        sites.get(1).setPercentage(2);
        sites.get(2).setPercentage(2);
        sites.get(3).setPercentage(2);
        sites.get(4).setPercentage(2);
        sites.get(5).setPercentage(5);
        sites.get(6).setPercentage(20);
        sites.get(7).setPercentage(20);
        sites.get(8).setPercentage(20);
        sites.get(9).setPercentage(20);
        sites.get(10).setPercentage(5);

        core.normalizeSites(sites);

        core.setSites(sites);
        core.setClipPolygon(boundingPolygon);
        core.voroDiagram();
    }

    public void update( float delta )
    {
        this.delta = delta;

        sites = core.getSites();

        core.iterateSimple();
        setClusterNodes();

//        calculatePos();
//        calculateForces();
//        ComputePowerDiagram();

        render.addSites( core.getSites() );
    }

    private void setClusterNodes()
    {
        for( Site s : sites )
        {
            for( ClusterNode node : clusternodes )
            {
                if((node.getPos().x == s.getOldX()) && (node.getPos().y == s.getOldY()))
                {
                    node.setPos(new Vector2(s.getX(), s.getY()));
                }
            }
        }
    }

    private void calculatePos()
    {
        Vector2[] oldpos = new Vector2[clusternodes.size()];

        //we will use a midpoint calculation to numerically solve the differential equation

        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            oldpos[i] = clusternodes.get(i).getPos();
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((clusternodes.get(i).getPos().x + (clusternodes.get(i).getVel().x * (delta/2))), (clusternodes.get(i).getPos().y + (clusternodes.get(i).getVel().y * (delta/2) ))));
        }
        calculateForces();
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((oldpos[i].x + (clusternodes.get(i).getVel().x * delta)), (oldpos[i].y + (clusternodes.get(i).getVel().y * delta ))));
        }
    }

    private void calculateForces()
    {
        for (ClusterNode node : clusternodes) {
            node.setForce(new Vector2(0, 0));
        }
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate
        for (ClusterEdge edge : clusteredges) {
            edge.ApplyForces();
        }

        for( ClusterNode node : clusternodes )
        {
            node.ApplyForces( clusternodes, delta );
        }
    }

    private void ComputePowerDiagram()
    {
        polys.clear();

        PowerDiagram diagram = new PowerDiagram();

        diagram.setSites(sites);
        diagram.setClipPoly(boundingPolygon);
        diagram.computeDiagram();

        voredges.clear();
        for (int i=0;i<sites.size;i++)
        {
            Site site=sites.array[i];
            PolygonSimple polygon=site.getPolygon();
            for( int j = 0; j < polygon.length; j++ )
            {
                if( j != (polygon.length-1)) {
                    voredges.add(new VoronoiEdge(new Vector2(polygon.getXPoints()[j], polygon.getYPoints()[j]), new Vector2(polygon.getXPoints()[j + 1], polygon.getYPoints()[j + 1])));
                }
                else
                {
                    voredges.add(new VoronoiEdge(new Vector2(polygon.getXPoints()[j], polygon.getYPoints()[j]), new Vector2(polygon.getXPoints()[0], polygon.getYPoints()[0])));
                }
            }
            polys.add(polygon);
        }
        render.addVoronoiEdges(voredges);
        render.addSites(sites);
    }

    private void AdaptPositionsWeights()
    {
        for( int i = 0; i < sites.size; i++ )
        {
            PolygonSimple poly = sites.get(i).getPolygon();
            if (poly != null) {
                double centroidX = poly.getCentroid().getX();
                double centroidY = poly.getCentroid().getY();
                sites.get(i).setXY(centroidX, centroidY);
            }
        }
    }

    private void AdaptWeights()
    {
        double averageDistance = getGlobalAvgNeighbourDistance();
        for( int i = 0; i < sites.size; i++ )
        {
            PolygonSimple poly = sites.get(i).getPolygon();

            double completeArea = boundingPolygon.getArea();

            double ACurrent = poly.getArea();
            double ATarget = completeArea*sites.get(i).getPercentage();

            System.out.println("percentage: " + sites.get(i).getPercentage() );

            double fAdapt = ATarget / ACurrent;

            double weightNew = Math.sqrt(sites.get(i).getWeight())*fAdapt;
            double wMax = averageDistance;

            if( weightNew > wMax )
            {
                sites.get(i).setWeight(wMax*wMax);
            }
            else
            {
                sites.get(i).setWeight(weightNew*weightNew);
            }
        }
    }

    private double getGlobalAvgNeighbourDistance() {
        double avg = 0;
        int num = 0;
        for (Site point : sites)
            if (point.getNeighbours() != null)
                for (Site neighbour : point.getNeighbours()) {
                    double distance = neighbour.distance(point);
                    avg += distance;
                    num++;
                }
        avg /= num;
        return avg;
    }

    private double computeAreaError() {
        double completeArea = boundingPolygon.getArea();
        double errorArea = 0;
        for (int z = 0; z < sites.size; z++) {
            Site point = sites.array[z];
            PolygonSimple poly = point.getPolygon();
            double currentArea = (poly == null) ? 0.0 : poly.getArea();
            double wantedArea = completeArea * point.getPercentage();
            errorArea += Math.abs(wantedArea - currentArea)
                    / (completeArea * 2.0);
        }
        return errorArea;
    }

    private double pDistance( Vector2 point, VoronoiEdge edge ) {

        double A = point.x - edge.getSource().x;
        double B = point.y - edge.getSource().y;
        double C = edge.getDest().x - edge.getSource().x;
        double D = edge.getDest().y - edge.getSource().y;

        double dot = ((A * C) + (B * D));
        double len_sq = ((C * C) + (D * D));
        double param = -1;
        if (len_sq != 0) //in case of 0 length line
        {
            param = dot / len_sq;
        }

        double xx, yy;

        if (param < 0) {
            xx = edge.getSource().x;
            yy = edge.getSource().y;
        }
        else if (param > 1) {
            xx = edge.getDest().x;
            yy = edge.getDest().y;
        }
        else {
            xx = edge.getSource().x + param * C;
            yy = edge.getSource().y + param * D;
        }

        double dx = point.x - xx;
        double dy = point.y - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

}
