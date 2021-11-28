package org.firstinspires.ftc.teamcode.controllers.arm;

import org.firstinspires.ftc.teamcode.hardware.Potentiometer;
import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ShoulderController {

    private double targetPos;
    private double currentVelo;
    private Potentiometer potentiometer;
    private DcMotor motor;


    public ShoulderController(Potentiometer potentiometer, DcMotor motor) {
        this.potentiometer = potentiometer;
    }

    public void update() {
        double angle = potentiometer.getAngleDegrees();
        if (angle < SHOULDER_MIN_ANGLE_DEGREES) {
            motor.setPower(1);
            return;
        } else if (angle >= SHOULDER_MAX_ANGLE_DEGREES) {
            motor.setPower(-1);
            return;
        }

        // Check for range
        if (Math.abs(targetPos - angle) <= SHOULDER_STALL_RANGE) {
            motor.setPower(SHOULDER_P_GAIN*(targetPos - angle));
        }

    }
}
