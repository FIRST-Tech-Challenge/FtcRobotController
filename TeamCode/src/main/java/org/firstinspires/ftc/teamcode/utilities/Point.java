/**
 * Created by EvanCoulson on 9/1/18.
 */

public class Point
{
    private double x;
    private double y;

    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double getX()
    {
        return x;
    }

    public double getY() {
        return y;
    }

    public static Point getOriginPoint() {
        return new Point(0,0);
    }
}
