package Tue.load;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Vector2
{
    public float x, y;

    public Vector2( float x, float y )
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

    public float distance(Vector2 v)
    {
        float a = v.x - x;
        float b = v.y - y;

        return (float)Math.sqrt( a * a + b * b);

    }
}