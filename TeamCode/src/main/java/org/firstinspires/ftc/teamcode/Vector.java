package org.firstinspires.ftc.teamcode;


public class Vector
{
    private double x = 0;
    private double y = 0;

    public void setCartesian(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public void setPolar(double mag, double dir)
    {
        clear();
        addPolar(mag, dir);
    }

    public void rotateDegrees(double dir)
    {
        setPolar(getMag(), getDir()+dir);
    }

    public void add(org.firstinspires.ftc.teamcode.Vector vector)
    {
        this.x += vector.x;
        this.y += vector.y;
    }

    public void addCartesian(double x, double y)
    {
        this.x += x;
        this.y += y;
    }

    public void addPolar(double mag, double dir)
    {
        double x = Math.cos(Math.toRadians(dir)) * mag;
        double y = Math.sin(Math.toRadians(dir)) * mag;

        addCartesian(x, y);
    }

    public void subtractCartesian(double x, double y)
    {
        this.x -= x;
        this.y -= y;
    }

    public void subtractPolar(double mag, double dir)
    {
        double x = Math.cos(Math.toRadians(dir)) * mag;
        double y = Math.sin(Math.toRadians(dir)) * mag;

        subtractCartesian(x, y);
    }

    public double getDir()
    {
        double theta = Math.toDegrees(Math.atan2(y, x));

        if(theta < 0)
        {
            theta += 360;
        }

        return theta;
    }

    public double getMag()
    {
        return Math.sqrt(x*x + y*y);
    }

    public double getX() { return x; }

    public double getY()
    {
        return y;
    }

    public void clear()
    {
        x = 0;
        y = 0;
    }

    public static double calcMag(double x, double y)
    {
        return Math.sqrt(x*x + y*y);
    }
}