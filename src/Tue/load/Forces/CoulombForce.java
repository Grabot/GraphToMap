package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterNode;

import java.util.ArrayList;

/**
 * Created by s138362 on 25-3-2016.
 */
public class CoulombForce
{

    private Vector2 posdif;

    public CoulombForce()
    {
        posdif = new Vector2(0, 0);
    }

    public void ApplyForces( ClusterNode source, ArrayList<ClusterNode> nodes )
    {
        float kC = 100f;
        float rSquared = 0;
        float forceX = 0;
        float forceY = 0;

        for( ClusterNode target : nodes )
        {
            if( source != target ) {
                posdif.x = source.getPos().x - target.getPos().x;
                posdif.y = source.getPos().y - target.getPos().y;

                rSquared = (posdif.x*posdif.x + posdif.y*posdif.y)+0.00001f;

                forceX = ((kC * posdif.x)/rSquared);
                forceY = ((kC * posdif.y)/rSquared);

                source.setForce( new Vector2( source.getForce().x + forceX, source.getForce().y + forceY ));
                target.setForce( new Vector2( target.getForce().x - forceX, target.getForce().y - forceY ));
            }
        }
    }

}
