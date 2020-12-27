package org.firstinspires.ftc.teamcode;

/**
 * Class DirectionDistance represents a Direction/Angle and Distance/length/Magnitude
 *
 * Here angle is measured Counter Clockwise (CCW) from the Y-Axis
 *
 * In the graph below  Point(4.0,4.0) =  DirectionDistance(-45, 4.2426) with some rounding.
 *
 *   Y Axis
 *    |
 *    |
 *    |
 *    |
 *    |           + (x1=4,y1=4)
 *    | -45d   /
 *    |     /  L = 4.2426
 *    |  /
 *    +----------------------------------- X Axis
 *  (0,0) origin
 *
 **/
public class DirectionDistance {
    // static definition of the axis
    public static final DirectionDistance X_AXIS = new DirectionDistance(-90.0, 1.0);
    public static final DirectionDistance Y_AXIS = new DirectionDistance(0.0,1.0);

    protected double _distance = 0.0; // distance - assume in inches
    protected double _direction = 0.0; // direction in degrees  measured counter clockwise from the y-axis. The way the robot is facing

    // new from x,y coordinate.
    static public DirectionDistance FromXY( double x, double y) {
        final double distance = Math.sqrt(x*x + y*y);
        final double theta = Math.atan2(x, y); // CCW from x Axis
        // subtract PI/2 to make relative to y-axis rather than x-axis
        return new DirectionDistance( Math.toDegrees(theta - Math.PI/2.0), distance);
    }

    // make a copy
    public DirectionDistance(final DirectionDistance other) {
        this._direction = other._direction;
        this._distance = other._distance;
    }

    // make a new object with specific values
    public DirectionDistance(double direction, double distance ){
        _direction = direction;
        _distance = distance;
    }

    public double direction() { return _direction; }
    public double distance() { return _distance; }

    public void setDirection( double direction ) { _direction = direction; }
    public void setDistance( double distance ) { _distance = distance; }

    public void set(double direction, double distance) { _direction = direction; _distance = distance; }

    public Point asPoint() {
        double theta = Math.toRadians(_direction) + Math.PI/2.0;
        return new Point(Math.cos(theta)* _distance, Math.sin(theta)* _distance);
    }

    // add other to the end of this to get the sum of the two as a new object
    public DirectionDistance plus( final DirectionDistance other) {
        Point thisAsPoint = asPoint();
        thisAsPoint.add( other );
        return thisAsPoint.asDirectionDistance();
    }

    // subtract other from the end of this the result as a new object
    public DirectionDistance minus( final DirectionDistance other) {
        Point thisAsPoint = asPoint();
        thisAsPoint.subtract( other );
        return thisAsPoint.asDirectionDistance();
    }

    // helper for telemetry
    @Override
    public String toString(){
        return String.format("(Dir:%.04fd,L:%.04f)", _direction, _distance);
    }
}
