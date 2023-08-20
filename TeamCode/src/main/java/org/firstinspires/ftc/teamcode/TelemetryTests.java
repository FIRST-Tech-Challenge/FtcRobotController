package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This opmode is for testing, getting telemetry, not for driving a robot
 */
@Disabled
@TeleOp
public class TelemetryTests extends OpMode {
    double priorTime = 0.0; // for logging loop times
    // IMU (inertia measurement unit, SDK 8.1 and later method of using IMU
    IMU imu;
    Orientation or; // robots orientation (x,y,z angles) from IMU on Hub


    @Override
    public void init() {
        // map IMU
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        //or = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.RADIANS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "TELEMETRY TEST, Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-TELEMETRY TEST = %.05f",getRuntime());
    }

    @Override
    public void loop() {
        double loopTime;

        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        telemetry.addData("IMU First Angle =","%.03f",or.firstAngle);
        telemetry.addData("IMU Second Angle =","%.03f",or.secondAngle);
        telemetry.addData("IMU Third Angle =","%.03f",or.thirdAngle);

        //telemetry.addData("Right stick X", "%.03f, Y = %.03f",gamepad1.right_stick_x,gamepad1.right_stick_y);
        //telemetry.addData("Left stick X", "%.03f, Y = %.03f", gamepad1.left_stick_x,gamepad1.left_stick_y);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();
        RobotLog.d("SRA-TELEMETRY-loop-time = %.05f, loop = %.05f",priorTime,loopTime);
    }
}
