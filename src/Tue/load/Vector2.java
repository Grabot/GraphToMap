package Tue.load;

/**
 * Created by s138362 on 16-3-2016.
 * Class that implements a point in Euclidean space
 */
public class Vector2 implements Comparable<Vector2>
{
    public double x, y;
    private Vector2 comparatorPoint = null;

    public Vector2( double x, double y )
    {
        this.x = x;
        this.y = y;
    }

    public Vector2( )
    {
        x = y = 0;
    }

    public void set( Vector2 v )
    {
        this.x = v.x;
        this.y = v.y;
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public Vector2 add( Vector2 v )
    {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    public Vector2 sub( Vector2 v )
    {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    public void set(float x, float y, float z)
    {
        this.x = x;
        this.y = y;
    }

    public double distance(Vector2 v)
    {
        double a = v.x - x;
        double b = v.y - y;

        return Math.sqrt( a * a + b * b);

    }


    public void setComparatorPoint(Vector2 p) {
        this.comparatorPoint = p;
    }

    public static boolean isCounterclockwise(Vector2 a, Vector2 b, Vector2 c) {
        // determines if the angle formed by a -> b -> c is counterclockwise
        double signed_area_doubled = (b.x - a.x) * (c.y - a.y) - (b.y - a.y)
                * (c.x - a.x);
        return (signed_area_doubled > 0);
    }

    @Override
    public int compareTo(Vector2 p) {
        // compares two points in the plane according to the polar angle they
        // form with the comparator point of this object
        // if a comparator point is not provided, the default is the origin
        // of the plane
        if (comparatorPoint == null) {
            comparatorPoint = new Vector2(0, 0);
        }

        Double angle1 = comparatorPoint.getPolarAngle(this);
        Double angle2 = comparatorPoint.getPolarAngle(p);
        return angle1.compareTo(angle2);
    }

    public double getPolarAngle() {
        double arctan = Math.atan2(y, x);
        return (arctan >= 0) ? arctan : (Math.PI * 2 - arctan);
    }

    public double getPolarAngle(Vector2 p) {
        double x_n = p.x - x;
        double y_n = p.y - y;
        return new Vector2(x_n, y_n).getPolarAngle();
    }


}