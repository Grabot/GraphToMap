package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterNode;

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
        node.setVel( new Vector2((node.getVel().x * 0.9f), (node.getVel().y * 0.9f)));
    }
}

