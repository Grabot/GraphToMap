package Tue.objects;

import Tue.load.Vector2;

/**
 * Created by s138362 on 29-6-2016.
 */
public class PolygonEdge
{

    private Vector2 from;
    private Vector2 to;

    public PolygonEdge( Vector2 from, Vector2 to )
    {
        this.from = from;
        this.to = to;
    }

    public double distanceToEdge( Node n )
    {
        //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        //P1 = from = (x1, y1)
        //P2 = to = (x2, y2)
        //(x0, y0) = (n.getX(), n.getY())
        //distance(P1, P2, (x0, y0)) = (|(y2-y1)x0-(x2-x1)y0+x2y1-y2x1|)/(sqrt((y2-y1)^2+(x2-x1)^2))
        //distance(P1, P2, (x0, y0)) = 2A/b
        double numerator = 2*getArea(n);
        double denominator = from.distance(to);

        double h = (numerator/denominator);
        return h;
    }
    public double distanceToEdge( Vector2 pos )
    {
        //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        //P1 = from = (x1, y1)
        //P2 = to = (x2, y2)
        //(x0, y0) = (n.getX(), n.getY())
        //distance(P1, P2, (x0, y0)) = (|(y2-y1)x0-(x2-x1)y0+x2y1-y2x1|)/(sqrt((y2-y1)^2+(x2-x1)^2))
        //distance(P1, P2, (x0, y0)) = 2A/b
        double numerator = 2*getArea(pos);
        double denominator = from.distance(to);

        double h = (numerator/denominator);
        return h;
    }

    public Vector2 getFrom()
    {
        return from;
    }

    public Vector2 getTo()
    {
        return to;
    }

    public double getArea( Node n )
    {
        //formula for area of a triangle given 3 points
        //http://www.mathopenref.com/coordtrianglearea.html
        double Ax = n.getPos().getX();
        double Ay = n.getPos().getY();
        double Bx = from.getX();
        double By = from.getY();
        double Cx = to.getX();
        double Cy = to.getY();

        double area = Math.abs((((Ax*(By-Cy))+(Bx*(Cy-Ay))+(Cx*(Ay-By)))/2));

        return area;
    }

    public double getArea( Vector2 pos )
    {
        //formula for area of a triangle given 3 points
        //http://www.mathopenref.com/coordtrianglearea.html
        double Ax = pos.getX();
        double Ay = pos.getY();
        double Bx = from.getX();
        double By = from.getY();
        double Cx = to.getX();
        double Cy = to.getY();

        double area = Math.abs((((Ax*(By-Cy))+(Bx*(Cy-Ay))+(Cx*(Ay-By)))/2));

        return area;
    }

}
