package Tue.load;

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

    private float delta = 0;

    public Simulation(ArrayList<ClusterNode> clusternodes, ArrayList<ClusterEdge> clusteredges)
    {
        this.clusternodes = clusternodes;
        this.clusteredges = clusteredges;
    }

    public void update( float delta )
    {
        this.delta = delta;

        calculatePos();
        calculateForces();
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

}
