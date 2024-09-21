package org.firstinspires.ftc.teamcode.core.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Drive;

public class Mecanum {
    static LinearOpMode _linearOpMode;

    public static  void initialize(LinearOpMode linearOpMode){
        _linearOpMode = linearOpMode;
        Drive.initialize(linearOpMode.hardwareMap, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IMU.initialization(linearOpMode.hardwareMap);
    }

    public static void driveToPosition(double power, int position) {
        Drive.setTargetPosition(position);
        Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drive.setPower(power);
        while (Drive.isBusy() && !_linearOpMode.isStopRequested()) { }
        Drive.stop();
    }

    public static void turnToDegree(double power, double targetDegrees){
        IMU.turnToDegree(power, targetDegrees);
    }

    public static void strafeToEncoder(double power, int position) {
        Drive.setTargetPosition(-position, position, position, -position);
        Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive.setPower(
                position > 0 ? -power : power,
                position > 0 ? power : -power,
                position > 0 ? power : -power,
                position > 0 ? -power : power
        );
        while (!Drive.isAtEncoder() && !_linearOpMode.isStopRequested()) { }
        Drive.stop();
    }

    public static void loggingAngles(double targetDegrees){
        IMU.loggingAngles(targetDegrees);
    }
}
