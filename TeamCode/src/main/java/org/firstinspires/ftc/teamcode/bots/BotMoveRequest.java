package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;

public class BotMoveRequest {
    private double topSpeed = 0;
    private RobotDirection direction;
    private Point target = null;
    private MotorReductionBot motorReduction =null;

    public double getTopSpeed() {
        return topSpeed;
    }

    public void setTopSpeed(double topSpeed) {
        this.topSpeed = topSpeed;
    }

    public RobotDirection getDirection() {
        return direction;
    }

    public void setDirection(RobotDirection direction) {
        this.direction = direction;
    }

    public Point getTarget() {
        return target;
    }

    public void setTarget(Point target) {
        this.target = target;
    }

    public MotorReductionBot getMotorReduction() {
        return motorReduction;
    }

    public void setMotorReduction(MotorReductionBot motorReduction) {
        this.motorReduction = motorReduction;
    }
}
