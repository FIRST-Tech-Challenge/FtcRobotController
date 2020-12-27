package org.firstinspires.ftc.teamcode;

/**
 * Class Point represents an x and y value.
 *
 * In a context where you have only one point to work with you can
 * think of the point as absolute relative to the origin or you graph
 *
 * EX: Point(x,y) = (x,y) - (0,0)
 * or simply Point(x,y) = (x,y)
 *
 *  Y Axis
 *    |
 *    |
 *    |
 *    |
 *    |           + (x1=4,y1=4)
 *    |        /
 *    |     /
 *    |  /
 *    +----------------------------------- X Axis
 *  (0,0) origin
 *
 * Based on current odometry where direction 0 is defined as
 * Forward direction of the robot aligns with the Y-Axis
 * and horizontal (strafe right) aligns with he X-Axis this class
 * makes the assumption that direction from DirectionDistance is
 * measured in Degrees with 0 at the Y-Axis and
 * + = CCW (Counter Clockwise), - = CW (Clockwise)
 *
 * Math functions like Math.sin and Math.cos require angles in Radian measured CCW
 * from the X-axis so in places where these calculations are performed we will convert
 * the units and apply a + or - PI/2 to shift the result appropriate to the situation.
 *
 **/
public class Point {

    // static definition of the axis
    public static final Point X_AXIS = new Point(1.0,0.0);
    public static final Point Y_AXIS = new Point(0.0,1.0);

    protected double _x = 0.0, _y = 0.0;

    // make a copy
    public Point(final Point other) {
        _x = other._x;
        _y = other._y;
    }

    // make a new object with specific values
    public Point(double x, double y ){
        _x = x;
        _y = y;
    }

    public Point set(final Point other) { _x = other._x; _y = other._y; return this; }

    // get the x or y value
    public double x() { return _x; }
    public double y() { return _y; }

    public void setX(double x ) { _x = x; }
    public void setY(double y ) { _y = y; }

    // make a new direction distance object from this Point
    public DirectionDistance asDirectionDistance() {
        return DirectionDistance.FromXY(_x, _y);
    }
    // assign x and y based on direction distance calculation.
    public void set(final DirectionDistance dd) {
        set( dd.asPoint() );
    }

    // to add another point to this point
    public void add( final Point other) { this._x += other._x; this._y += other._y; }

    // to add a DirectionDistance to this point.
    public void add( final DirectionDistance dd) {
        Point ddAsPoint = dd.asPoint();
        this._x += ddAsPoint._x; this._y += ddAsPoint._y;
    }

    // to calculate a new point by adding this point with another point
    public Point plus( final Point other) {
        return new Point( this._x + other._x, this._y + other._y );
    }

    // to subtract another point from this point
    public void subtract( final Point other) { this._x -= other._x; this._y -= other._y; }

    // to subtract a direction distance from to this point
    public void subtract( final DirectionDistance dd ) {
        Point ddAsPoint = dd.asPoint();
        this._x -= ddAsPoint._x; this._y -= ddAsPoint._y;
    }

    // to calculate a new point by subtracting another point from this point
    public Point minus( final Point other) {
        return new Point( this._x - other._x, this._y - other._y );
    }

    // to calculate a new point by subtracting a direction distance from to this point
    public Point minus( final DirectionDistance dd ) {
        Point ddAsPoint = dd.asPoint();
        return new Point(this._x - ddAsPoint._x, this._y - ddAsPoint._y );
    }

    // helper for telemetry formats the point as a string
    @Override
    public String toString(){
        return String.format("(x:%.04f,y:%.04f)", _x, _y);
    }
}
