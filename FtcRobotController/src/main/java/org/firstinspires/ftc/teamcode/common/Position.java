package org.firstinspires.ftc.teamcode.common;

public class Position {
    private Distance x;
    private Distance y;
    private Angle theta;
    public Position(Distance x, Distance y, Angle theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Position add(Distance x, Distance y, Angle theta) {
        this.x.add(x);
        this.y.add(y);
        this.theta.add(theta);
        return this;
    }

    public Position copy(){
        return new Position(x.copy(),y.copy(),theta.copy());
    }

    public Distance getX() {
        return x;
    }

    public Distance getY() {
        return y;
    }

    public Angle getTheta() {
        return theta;
    }

    public void setY(Distance y)
    {
        this.y = y;
    }

    public void setX(Distance x)
    {
        this.x = x;
    }

    public void setTheta(Angle theta) {
        this.theta = theta;
    }

    @Override
    public String toString() {
        return "x: "+ x.toInches() + ", y: " + y.toInches() + ", theta: " + theta.toDegrees();
    }

}
