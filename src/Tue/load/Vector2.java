package Tue.load;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Vector2
{
    public double x, y;

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
}