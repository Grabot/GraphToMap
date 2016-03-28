package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterNode;

/**
 * Created by s138362 on 25-3-2016.
 */
public class WallForce
{

    private int width = 0;
    private int height = 0;

    public WallForce( int width, int height )
    {
        this.width = width;
        this.height = height;
    }

    public void ApplyForces( ClusterNode node, float delta )
    {
        float k = 100;
        float posdif = 0;
        //check if it will be through the wall in a next step
        //to the left side of the wall
        if((node.getPos().x + node.getVel().x * delta) < 0)
        {
            posdif = (node.getPos().x + node.getVel().x * delta);
            node.setForce( new Vector2( node.getForce().x - k*posdif, node.getForce().y ));
        }

        if((node.getPos().x + node.getVel().x * delta) > width )
        {
            posdif = ((node.getPos().x + node.getVel().x * delta) - width);
            node.setForce( new Vector2( node.getForce().x - k*posdif, node.getForce().y ));
        }

        if((node.getPos().y + node.getVel().y * delta) < 0)
        {
            posdif = (node.getPos().y + node.getVel().y * delta);
            node.setForce( new Vector2( node.getForce().x, node.getForce().y - k*posdif ));
        }

        if((node.getPos().y + node.getVel().y * delta) > height )
        {
            posdif = ((node.getPos().y + node.getVel().y * delta) - height);
            node.setForce( new Vector2( node.getForce().x, node.getForce().y - k*posdif ));
        }
    }

}
