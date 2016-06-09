package Tue.load;

import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.objects.Node;

/**
 * Created by s138362 on 8-6-2016.
 */
public class LabelObject
{

    private Node node;
    private PolygonSimple rect;
    private double nodeValue;

    public LabelObject(Node node, PolygonSimple rect, double nodeValue )
    {
        this.node = node;
        this.rect = rect;
        this.nodeValue = nodeValue;
    }

    public double getNodeValue()
    {
        return nodeValue;
    }

    public Node getNode()
    {
        return node;
    }

    public void setRect(PolygonSimple rect)
    {
        this.rect = rect;
    }

    public PolygonSimple getRect()
    {
        return rect;
    }

}
