package Tue.load.Forces;

import Tue.load.Vector2;
import Tue.objects.ClusterEdge;

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
        float ks = 3;
        posdif.x = edge.getSource().getPos().x - edge.getDest().getPos().x;
        posdif.y = edge.getSource().getPos().y - edge.getDest().getPos().y;

        veldif.x = edge.getSource().getVel().x - edge.getDest().getVel().x;
        veldif.y = edge.getSource().getVel().y - edge.getDest().getVel().y;

        double posLength = (Math.sqrt(((posdif.x*posdif.x) + (posdif.y*posdif.y))));
        float dotProduct = ((posdif.x*veldif.x) + (posdif.y + veldif.y));

        force.x = (posdif.x/(float)posLength)*((ks * ((float)posLength - (float)edge.getWeight())) + (ks * ( dotProduct / (float)posLength)));
        force.y = (posdif.y/(float)posLength)*((ks * ((float)posLength - (float)edge.getWeight())) + (ks * ( dotProduct / (float)posLength)));

        //apply forces, the same force is applied towards opposite direction of the nodes
        edge.getDest().setForce( new Vector2( edge.getDest().getForce().x + force.x, edge.getDest().getForce().y + force.y) );
        edge.getSource().setForce( new Vector2( edge.getSource().getForce().x - force.x, edge.getSource().getForce().y - force.y) );
    }
}