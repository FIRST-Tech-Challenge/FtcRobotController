package org.firstinspires.ftc.teamcode.Navigation;

class Velocity {
    final double x;
    final double y;
    final double theta;

    Velocity(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
//add vector to vector
    public Velocity add (Velocity velocity) {
        return new Velocity(
          this.x + velocity.x,
          this.y + velocity.y,
          this.theta + velocity.y
        );
    }

    @Override
    public String toString() {
        return "{" +
                "x=" + x +
                ", y=" + y +
                ", theta=" + theta +
                '}';
    }
}