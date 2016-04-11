package Tue.objects;

/**
 * Created by s138362 on 10-4-2016.
 */
public class DelaunayFace
{

    private Cluster A;
    private Cluster B;
    private Cluster C;

    private boolean negative = false;

    public DelaunayFace( Cluster A, Cluster B, Cluster C )
    {
        this.A = A;
        this.B = B;
        this.C = C;

        if( getArea() < 0 )
        {
            negative = true;
        }
    }

    public Cluster getA()
    {
        return A;
    }

    public Cluster getB()
    {
        return B;
    }

    public Cluster getC()
    {
        return C;
    }

    public double getArea()
    {
        //formula for area of a triangle given 3 points
        //http://www.mathopenref.com/coordtrianglearea.html
        double Ax = A.getPos().getX();
        double Ay = A.getPos().getY();
        double Bx = B.getPos().getX();
        double By = B.getPos().getY();
        double Cx = C.getPos().getX();
        double Cy = C.getPos().getY();

        double area = (((Ax*(By-Cy))+(Bx*(Cy-Ay))+(Cx*(Ay-By)))/2);
        if( negative )
        {
            area = (area*-1);
        }

        return area;
    }

    public double[] getAngles()
    {
        //solving sss triangle, side side side
        //https://www.mathsisfun.com/algebra/trig-solving-sss-triangles.html
        //cos(A) = (b^2 + c^2 - a^2)/2*c*a
        //cos(B) = (a^2 + c^2 - b^2)/2*a*c
        //cos(C) = (a^2 + b^2 - c^2)/2*a*b

        double[] angles = new double[3];

        double sideA = B.getPos().distance(C.getPos());
        double sideB = A.getPos().distance(C.getPos());
        double sideC = A.getPos().distance(B.getPos());

        double cosA = (((sideB*sideB)+(sideC*sideC)-(sideA*sideA))/(2*sideB*sideC));
        double cosB = (((sideA*sideA)+(sideC*sideC)-(sideB*sideB))/(2*sideA*sideC));
        double cosC = (((sideB*sideB)+(sideA*sideA)-(sideC*sideC))/(2*sideB*sideC));

        angles[0] = Math.acos(cosA)*(180/Math.PI);
        angles[1] = Math.acos(cosB)*(180/Math.PI);
        angles[2] = Math.acos(cosC)*(180/Math.PI);

        return angles;
    }
}
