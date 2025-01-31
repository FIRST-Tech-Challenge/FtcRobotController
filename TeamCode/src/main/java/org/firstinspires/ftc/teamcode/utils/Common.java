package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;
import java.util.OptionalDouble;

public class Common {


    public static double degreesToTicks(double degrees) {
        final double ARM_TICKS_PER_DEGREE = 28.0 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
        return degrees * ARM_TICKS_PER_DEGREE;
    }
    public static double slideServoMMToTicks(double mm) {
        final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
        return mm * LIFT_TICKS_PER_MM;
    }
    public static Optional<DcMotorEx> convertToDcMotorExOrWarn(DcMotor motor, Telemetry telemetry, String motorName) {
        if (motor instanceof DcMotorEx) {
            return Optional.of((DcMotorEx) motor);
        }
        telemetry.addLine("WARNING: " + motorName + " motor is not a DcMotorEx");
        return Optional.empty();
    }
    public static void warnIfOvercurrent(DcMotorEx motor, Telemetry telemetry, String motorName) {
        if (motor.isOverCurrent()) {
            telemetry.addLine("WARNING: " + motorName + " motor is over current");
        }
    }
    public static double cycleTime;
    public static double loopTime;
    public static double oldTime;
    public static double slidePosition = 0;
    public static double armPosition = 0;
    public static void updateCycleTimes(double runtime) {
        loopTime = runtime;
        cycleTime = loopTime - oldTime;
        oldTime = loopTime;
    }
    public static OptionalDouble yawAngle = OptionalDouble.empty();

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

