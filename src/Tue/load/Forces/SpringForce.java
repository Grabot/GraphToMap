package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterEdge;
import Tue.objects.DelaunayEdge;
import Tue.objects.Edge;
import Tue.objects.TestEdge;

/**
 * Created by s138362 on 24-3-2016.
 */
public class SpringForce
{
    private Vector2 posdif;
    private Vector2 veldif;
    private Vector2 force;

    public SpringForce()
    {
        posdif = new Vector2(0, 0);
        veldif = new Vector2(0, 0);
        force = new Vector2(0, 0);
    }

    //hooke's law for spring forces
    public void ApplyForce( ClusterEdge edge )
    {
        double ks = 3;
        posdif.x = edge.getSource().getPos().x - edge.getDest().getPos().x;
        posdif.y = edge.getSource().getPos().y - edge.getDest().getPos().y;

        veldif.x = edge.getSource().getVel().x - edge.getDest().getVel().x;
        veldif.y = edge.getSource().getVel().y - edge.getDest().getVel().y;

        double posLength = (Math.sqrt(((posdif.x*posdif.x) + (posdif.y*posdif.y))));
        double dotProduct = ((posdif.x*veldif.x) + (posdif.y + veldif.y));

        force.x = (posdif.x/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));
        force.y = (posdif.y/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));

        //apply forces, the same force is applied towards opposite direction of the nodes
        edge.getDest().setForce( new Vector2( edge.getDest().getForce().x + force.x, edge.getDest().getForce().y + force.y) );
        edge.getSource().setForce( new Vector2( edge.getSource().getForce().x - force.x, edge.getSource().getForce().y - force.y) );
    }

    public void ApplyForce( DelaunayEdge edge)
    {
        double ks = 3.8;
        posdif.x = edge.getSource().getPos().x - edge.getDest().getPos().x;
        posdif.y = edge.getSource().getPos().y - edge.getDest().getPos().y;

        veldif.x = edge.getSource().getVel().x - edge.getDest().getVel().x;
        veldif.y = edge.getSource().getVel().y - edge.getDest().getVel().y;

        double posLength = (Math.sqrt(((posdif.x*posdif.x) + (posdif.y*posdif.y))));
        double dotProduct = ((posdif.x*veldif.x) + (posdif.y + veldif.y));

        force.x = (posdif.x/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));
        force.y = (posdif.y/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));

        //apply forces, the same force is applied towards opposite direction of the nodes
        edge.getDest().setForce( new Vector2( edge.getDest().getForce().x + force.x, edge.getDest().getForce().y + force.y) );
        edge.getSource().setForce( new Vector2( edge.getSource().getForce().x - force.x, edge.getSource().getForce().y - force.y) );
    }

    public void ApplyForce( Edge edge )
    {
        double ks = 0.15;
        posdif.x = edge.getSource().getPos().x - edge.getDest().getPos().x;
        posdif.y = edge.getSource().getPos().y - edge.getDest().getPos().y;

        veldif.x = edge.getSource().getVel().x - edge.getDest().getVel().x;
        veldif.y = edge.getSource().getVel().y - edge.getDest().getVel().y;

        double posLength = (Math.sqrt(((posdif.x*posdif.x) + (posdif.y*posdif.y))));
        double dotProduct = ((posdif.x*veldif.x) + (posdif.y + veldif.y));

        if( posLength != 0 )
        {
            force.x = (posdif.x/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));
            force.y = (posdif.y/posLength)*((ks * (posLength - edge.getWeight())) + (ks * ( dotProduct / posLength)));
        }
        else
        {
            force.x = 0;
            force.y = 0;
        }

        //apply forces, the same force is applied towards opposite direction of the nodes
        edge.getDest().setForce( new Vector2( edge.getDest().getForce().x + force.x, edge.getDest().getForce().y + force.y) );
        edge.getSource().setForce( new Vector2( edge.getSource().getForce().x - force.x, edge.getSource().getForce().y - force.y) );
    }

    public void ApplyForce( TestEdge edge)
    {
        double ks = 1;
        posdif.x = edge.getSource().getX() - edge.getDest().getPos().x;
        posdif.y = edge.getSource().getY() - edge.getDest().getPos().y;

        double posLength = (Math.sqrt(((posdif.x*posdif.x) + (posdif.y*posdif.y))));

        force.x = (posdif.x/posLength)*((ks * (posLength - edge.getWeight())));
        force.y = (posdif.y/posLength)*((ks * (posLength - edge.getWeight())));

        //apply forces, the same force is applied towards opposite direction of the nodes
        edge.getSource().setForce( new Vector2( edge.getSource().getForce().x - force.x, edge.getSource().getForce().y - force.y) );
    }

}
