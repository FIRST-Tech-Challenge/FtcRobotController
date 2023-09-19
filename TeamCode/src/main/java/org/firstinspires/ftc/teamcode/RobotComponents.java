package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotComponents {
    private final DcMotorEx motor;
    private final double ticks_per_degree;
    private final double F;
    private final PIDController controller;
    private double target;

    RobotComponents(DcMotorEx motor, double ticks_per_rotation, double p, double i, double d, double f) {
        this.motor = motor;
        this.F = f;
        ticks_per_degree = ticks_per_rotation / 360.0;
        target = 0;

        controller = new PIDController(p, i, d);
    }

    public double getTicksPerDegree() {
        return ticks_per_degree;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }

    public void incrementTarget(double increment) {
        target += increment;
        moveUsingPID();
    }

    public void moveUsingPID() {

        controller.reset();
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * F;
        double power = pid + ff;

        motor.setPower(power);

    }

    public boolean motorCloseEnough(int range) {
        return (target - range <= motor.getCurrentPosition()) && (target + range >= motor.getCurrentPosition());
    }
}