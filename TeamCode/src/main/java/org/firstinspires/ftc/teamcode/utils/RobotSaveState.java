package org.firstinspires.ftc.teamcode.utils;

public class RobotSaveState {
    private double horizontalPower;
    private double verticalPower;
    private double rotationalPower;

    public RobotSaveState(double horizontalPower, double verticalPower, double rotationalPower) {
        this.horizontalPower = horizontalPower;
        this.verticalPower = verticalPower;
        this.rotationalPower = rotationalPower;
    }

    public double getRotationalPower() {
        return rotationalPower;
    }

    public void setRotationalPower(double rotationalPower) {
        this.rotationalPower = rotationalPower;
    }

    public double getVerticalPower() {
        return verticalPower;
    }

    public void setVerticalPower(double verticalPower) {
        this.verticalPower = verticalPower;
    }

    public double getHorizontalPower() {
        return horizontalPower;
    }

    public void setHorizontalPower(double horizontalPower) {
        this.horizontalPower = horizontalPower;
    }
}
