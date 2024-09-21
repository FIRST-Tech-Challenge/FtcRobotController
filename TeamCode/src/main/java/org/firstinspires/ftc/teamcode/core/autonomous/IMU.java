package org.firstinspires.ftc.teamcode.core.autonomous;

import android.util.Log;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.Drive;

public class IMU {
    static BHI260IMU.Parameters _parameters;
    static BHI260IMU _imu;
    static Orientation _angles;
    public static void initialization(HardwareMap hardwareMap) {
        _parameters = new com.qualcomm.robotcore.hardware.IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        _imu = hardwareMap.get(BHI260IMU.class, "imu");
        _imu.initialize(_parameters);
    }
    public static void loggingAngles(double targetDegrees){
        _angles = _imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.d("5960", "First Angle: " + _angles.firstAngle);
        Log.d("5960", "Second Angle: " + _angles.secondAngle);
        Log.d("5960", "Third Angle: " + _angles.thirdAngle);
    }
    public static void turnToDegree(double power, double targetDegrees) {
        Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _angles = _imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentDegrees = AngleUnit.DEGREES.fromUnit(_angles.angleUnit, _angles.firstAngle);

        if (targetDegrees < currentDegrees) {
            Drive.setPower(-power, power, -power, power);
            while (targetDegrees < currentDegrees) {
                _angles = _imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentDegrees = AngleUnit.DEGREES.fromUnit(_angles.angleUnit, _angles.firstAngle);
            }
            Drive.stop();
        } else if (targetDegrees > currentDegrees) {
            Drive.setPower(power, -power, power, -power);
            while (targetDegrees > currentDegrees) {
                _angles = _imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentDegrees = AngleUnit.DEGREES.fromUnit(_angles.angleUnit, _angles.firstAngle);
            }
            Drive.stop();
        }
    }
}
