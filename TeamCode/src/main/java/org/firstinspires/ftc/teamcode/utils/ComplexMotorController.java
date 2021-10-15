package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Represents a complex control plane for motors connected to the robot.
 * @author Thomas Ricci
 */
@Deprecated
public class ComplexMotorController {

    private final Telemetry TELEMETRY;

    /**
     * Creates a complex control plane.
     * @param telemetry The telemetry object to log to.
     */
    public ComplexMotorController(Telemetry telemetry) {
        TELEMETRY = telemetry;
    }

    /**
     * Prepares a motor to be ran.
     * @param motor The motor to prepare.
     * @param distance The distance the motor will travel over the next run.
     */
    public void prepMotor(EncoderMotor motor, double distance) {
        DcMotor dcMotor = motor.getDcMotor();
        dcMotor.setTargetPosition(dcMotor.getCurrentPosition() + (int)(distance * motor.getCountsPerInch()));
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TELEMETRY.addData("Motor ready",  "Position is " + dcMotor.getCurrentPosition());
        TELEMETRY.update();
    }

    /**
     * Starts running the motor at a specific speed.
     * @param motor The motor to run.
     * @param speed The speed at which to run the motor.
     */
    public void startMotor(EncoderMotor motor, double speed) {
        DcMotor dcMotor = motor.getDcMotor();
        dcMotor.setPower(Math.abs(speed));
        TELEMETRY.addData("Run started",  "Position is " + dcMotor.getCurrentPosition());
        TELEMETRY.update();
    }

    /**
     * Stops running the motor at a specific speed.
     * @param motor The motor to stop.
     */
    public void stopMotor(EncoderMotor motor) {
        DcMotor dcMotor = motor.getDcMotor();
        dcMotor.setPower(0);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TELEMETRY.addData("Run complete and motor ready for next run",  "Position is " + dcMotor.getCurrentPosition());
        TELEMETRY.update();
    }

}
