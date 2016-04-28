package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterNode;
import Tue.objects.Node;

/**
 * Created by s138362 on 25-3-2016.
 */
public class FrictionForce
{

    public FrictionForce()
    {

    }

    public void ApplyForces(ClusterNode node)
    {
        node.setVel( new Vector2((node.getVel().x * 0.8f), (node.getVel().y * 0.8f)));
    }

    public void ApplyForces( Node node )
    {
        node.setVel( new Vector2((node.getVel().x * 0.8f), (node.getVel().y * 0.8f)));
    }
}

