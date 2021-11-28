package org.firstinspires.ftc.teamcode.utils;

public class Pose2D {
    public double x, y, theta;

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2D(double x, double y) {
        this(x, y, 0d);
    }
    public Pose2D() {
        this(0d, 0d, 0d);
    }
    public Pose2D(Vector2D vector, double theta) {this(vector.x, vector.y, theta); }

    public Pose2D rotate(double dTheta) {
        theta += dTheta;
        return this;
    }

    public Pose2D scale(double scalar) {
        x *= scalar;
        y *= scalar;
        return this;
    }

    public Pose2D addPose(Pose2D other) {
        x += other.x;
        y += other.y;
        theta += other.theta;
        return this;
    }

    public Pose2D globalize(double heading) {
        double newX = Math.cos(heading)*this.x + Math.sin(heading)*this.y;
        this.y = -Math.sin(heading)*this.x + Math.cos(heading)*this.y;
        this.x = newX;
        this.theta = heading;
        return this;
    }

    public Pose2D copy() {
        return new Pose2D(this.x, this.y, this.theta);
    }

    public Pose2D addVector(Vector2D other) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }
}
