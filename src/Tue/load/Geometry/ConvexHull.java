package Tue.load.Geometry;


import Tue.load.Vector2;
import java.util.Arrays;

/**
 * Created by s138362 on 4-4-2016.
 */
public class ConvexHull {

    private int n;
    private Vector2[] points;

    public ConvexHull()
    {
    }

    public void setPoints( Vector2[] points )
    {
        this.n = points.length;
        this.points = points;
    }

    private Vector2 getPointWithLowestYCoord() {
        Vector2 lowest_point = points[0];
        for (int i = 1; i < n; i++) {
            if (points[i].getY() < lowest_point.getY()) {
                lowest_point = points[i];
            } else if (points[i].getY() < lowest_point.getY()) {
                if (points[i].getX() > lowest_point.getX())
                    lowest_point = points[i];
            }
        }
        return lowest_point;
    }

    private int computeConvexHull() {
        if (n <= 2)
            return n;
        Vector2 point_with_lowest_y_coord = getPointWithLowestYCoord();
        for (int i = 0; i < n; i++) {
            points[i].setComparatorPoint(point_with_lowest_y_coord);
        }
        Arrays.sort(points);
        Vector2[] mod_points = new Vector2[n + 1];
        for (int i = 0; i < n; i++) {
            mod_points[i] = points[i];
        }
        mod_points[n] = points[0];
        points = mod_points;
        int convex_hull_index = 1;
        int i = 2;
        while (i <= n) {
            // decide if an angle is counterclockwise or not
            // if it is not counterclockwise, do not include it in the
            // convex hull
            while (!Vector2.isCounterclockwise(points[convex_hull_index - 1],
                    points[convex_hull_index], points[i])) {
                if (convex_hull_index > 1)
                    convex_hull_index--;
                else if (i == n)
                    // all points are collinear
                    break;
                else
                    i++;
            }
            convex_hull_index++;
            swap(points, convex_hull_index, i);
            i++;
        }
        return convex_hull_index;
    }

    public Vector2[] getConvexHull() {
        int convex_hull_index = computeConvexHull();
        Vector2[] convex_hull_points = new Vector2[convex_hull_index];
        for (int i = 0; i < convex_hull_index; i++) {
            convex_hull_points[i] = points[i];
        }
        return convex_hull_points;
    }

    private void swap(Vector2[] points, int index1, int index2) {
        assert (index1 < points.length);
        assert (index2 < points.length);

        Vector2 aux = points[index1];
        points[index1] = points[index2];
        points[index2] = aux;
    }

}