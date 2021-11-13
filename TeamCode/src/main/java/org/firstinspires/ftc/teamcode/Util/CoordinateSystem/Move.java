package org.firstinspires.ftc.teamcode.Util.CoordinateSystem;

public class Move {
    private double distance;
    private double angle;
    private double turn;
    private double power;

    public Move(double distance, double angle, double turn, double power) {
        this.distance = distance;
        this.angle = angle;
        this.turn = turn;
        this.power = power;
    }

    public Move(double distance, double angle, double turn) {
        this(distance, angle, turn, 1);
    }

    public Move(double distance, double angle) {
        this(distance, angle, 0, 1);
    }

    public Move(double distance) {
        this(distance, 0, 0, 1);
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getTurn() {
        return turn;
    }

    public void setTurn(double turn) {
        this.turn = turn;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }
}
