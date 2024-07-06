package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DrivetrainState extends GenericState {
    public static final double ERROR_TOLERANCE = 15;

    private double currentTargetTicks[] = new double[4];

    private double currentTicks[] = new double[4];

    private LinearOpMode opMode;

    private double error[] = new double[4];

    public double getCurrentTargetTicks(int motorNumber) {
        return currentTargetTicks[motorNumber];
    }

    public void setCurrentTargetTicks(double currentTargetTicksFLeft, double currentTargetTicksFRight, double currentTargetTicksBLeft, double currentTargetTicksBRight) {
        this.currentTargetTicks[0] = currentTargetTicksFLeft;
        this.currentTargetTicks[1] = currentTargetTicksFRight;
        this.currentTargetTicks[2] = currentTargetTicksBLeft;
        this.currentTargetTicks[3] = currentTargetTicksBRight;

        this.error[0] = currentTargetTicks[0] - currentTicks[0];
        this.error[1] = currentTargetTicks[1] - currentTicks[1];
        this.error[2] = currentTargetTicks[2] - currentTicks[2];
        this.error[3] = currentTargetTicks[3] - currentTicks[3];
    }

    public double getCurrentTicks(int motorNumber) {
        return currentTicks[motorNumber];
    }

    public void setCurrentTicks(double currentTicksFLeft, double currentTicksFRight, double currentTicksBLeft, double currentTicksBRight) {
        this.currentTicks[0] = currentTicksFLeft;
        this.currentTicks[1] = currentTicksFRight;
        this.currentTicks[2] = currentTicksBLeft;
        this.currentTicks[3] = currentTicksBRight;

        this.error[0] = currentTargetTicks[0] - currentTicks[0];
        this.error[1] = currentTargetTicks[1] - currentTicks[1];
        this.error[2] = currentTargetTicks[2] - currentTicks[2];
        this.error[3] = currentTargetTicks[3] - currentTicks[3];
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public double getError(int motorNumber) {
        return error[motorNumber];
    }

    @Override
    boolean isDone() {
        if((Math.abs(error[0])<ERROR_TOLERANCE) && (Math.abs(error[1])<ERROR_TOLERANCE) && (Math.abs(error[2])<ERROR_TOLERANCE) && (Math.abs(error[3])<ERROR_TOLERANCE)) {
            return true;
        }

        return false;
    }
}
