package Tue.load;

/**
 * Created by s138362 on 6-4-2016.
 */
public class Positioning
{
    private double distortion = Double.MAX_VALUE;
    private double[][] pos;

    public Positioning( double[][] pos, double distortion )
    {
        this.distortion = distortion;
        this.pos = pos;
    }

    public double getDistortion()
    {
        return distortion;
    }

    public double[][] getPos()
    {
        return pos;
    }
}
