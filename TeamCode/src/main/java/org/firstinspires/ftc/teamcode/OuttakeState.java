package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OuttakeState extends GenericState{

    public static final double ERROR_TOLERANCE = 15;
    private double currentTargetTicks;
    private double currentTicks;

    private LinearOpMode opMode;

    private double error;

    public double getCurrentTargetTicks() {
        return currentTargetTicks;
    }

    public void setCurrentTargetTicks(double currentTargetTicks) {
        this.currentTargetTicks = currentTargetTicks;
        this.error = currentTargetTicks - currentTicks;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public void setCurrentTicks(double currentTicks) {
        this.currentTicks = currentTicks;
        this.error = currentTargetTicks - currentTicks;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public double getError() {
        return error;
    }

    @Override
    boolean isDone() {
        return Math.abs(error)<ERROR_TOLERANCE;
    }
}
