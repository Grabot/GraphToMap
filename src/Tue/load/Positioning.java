package Tue.load;

/**
 * Created by s138362 on 6-4-2016.
 */
public class Positioning
{
    private double distortion = Double.MAX_VALUE;
    private double[][] Clusterpos;
    private double[] normalPos;

    public Positioning( double[][] Clusterpos, double distortion )
    {
        this.distortion = distortion;
        this.Clusterpos = Clusterpos;
    }

    public Positioning( double[] normalPos, double distortion )
    {
        this.distortion = distortion;
        this.normalPos = normalPos;
    }

    public double getDistortion()
    {
        return distortion;
    }

    public double[][] getClusterPos()
    {
        return Clusterpos;
    }

    public double[] getNormalPos()
    {
        return normalPos;
    }
}
