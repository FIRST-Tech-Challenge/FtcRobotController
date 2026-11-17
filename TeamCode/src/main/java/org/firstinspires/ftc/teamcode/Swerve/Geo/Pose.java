package org.firstinspires.ftc.teamcode.Swerve.Geo;

public class Pose extends Point {

    public double heading;

    public Pose(double x, double y, double heading){
        super( x, y);
        this.heading = heading;
    }

    public Pose(Point p, double heading){
        this( p.x, p.y, heading);
    }

    public Pose(D2Vector v, double heading){
        this( v.x, v.y, heading);
    }

    public Pose(){
        this( 0, 0, 0);
    }
    public Pose sumPose(Pose other){
        return new Pose(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }
    public Pose subtractPose(Pose other){
        return new Pose(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    public Pose divPose(Pose other){
        return new Pose(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public Pose multPose(Pose other){
        return new Pose(this.x * other.x, this.y * other.y, this.heading * other.heading);
    }

    public Pose addPose(Pose other){
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose subPose(Pose other){
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose rotatePosePoint(Pose other, Double rotationAmount) {
        return new Pose(new Point(other.x,other.y).rotate(rotationAmount),other.heading);
    }
}
