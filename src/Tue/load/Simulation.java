package Tue.load;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.diagram.PowerDiagram;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;

import java.util.ArrayList;

/**
 * Created by s138362 on 27-3-2016.
 */
public class Simulation
{

    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
    private ArrayList<PolygonSimple> polys = new ArrayList<PolygonSimple>();

    private float delta = 0;

    private int width;
    private int height;

    private PolygonSimple boundingPolygon;
    private OpenList sites;
    private PowerDiagram diagram;

    private Renderer render;

    public Simulation(Renderer render, ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges, int width, int height )
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
        this.render = render;

        diagram = new PowerDiagram();

        // normal list based on an array
        sites = new OpenList();

        //create bounding polygon (the size of the panel)
        boundingPolygon = new PolygonSimple();
        boundingPolygon.add(0, 0);
        boundingPolygon.add(width, 0);
        boundingPolygon.add(width, height);
        boundingPolygon.add(0, height);
        render.addBounding(boundingPolygon);

        //define the clusternodes as sites for the powerdiagram
        for (int i = 0; i < clusternodes.size(); i++) {
            Site site = new Site(clusternodes.get(i).getPos().x, clusternodes.get(i).getPos().y);
            // we could also set a different weighting to some sites
            // site.setWeight(30)
            sites.add(site);
        }

        // set the list of points (sites), necessary for the power diagram
        diagram.setSites(sites);
        // set the clipping polygon, which limits the power voronoi diagram
        diagram.setClipPoly(boundingPolygon);

        // do the computation
        diagram.computeDiagram();
        // for each site we can no get the resulting polygon of its cell.
        // note that the cell can also be empty, in this case there is no polygon for the corresponding site.
        for (int i=0;i<sites.size;i++)
        {
            Site site=sites.array[i];
            PolygonSimple polygon=site.getPolygon();
            render.addVoronoiArea( polygon );
            polys.add(polygon);
        }

    }

    public void update( float delta )
    {
        this.delta = delta;

        calculatePos();
        calculateForces();
        calculateVoronoiArea();
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

    private void calculateVoronoiArea()
    {
        polys.clear();
        sites.clear();

        for (int i = 0; i < clusternodes.size(); i++) {
            Site site = new Site(clusternodes.get(i).getPos().x, clusternodes.get(i).getPos().y);
            // we could also set a different weighting to some sites
            // site.setWeight(30)
            sites.add(site);
        }

        // set the list of points (sites), necessary for the power diagram
        diagram.setSites(sites);
        // set the clipping polygon, which limits the power voronoi diagram
        diagram.setClipPoly(boundingPolygon);

        // do the computation
        diagram.computeDiagram();
        // for each site we can no get the resulting polygon of its cell.
        // note that the cell can also be empty, in this case there is no polygon for the corresponding site.
        render.clearVoronoi();
        for (int i=0;i<sites.size;i++)
        {
            Site site=sites.array[i];
            PolygonSimple polygon=site.getPolygon();
            render.addVoronoiArea( polygon );
            polys.add(polygon);
        }
    }

}
