package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.annotation.ElementType;


@Autonomous(group="chrisBot", name="PID Drive Test with Objects")

public class ObjectPIDDrive extends LinearOpMode {
    chrisBot robot = new chrisBot();
    PID pidDrive;
    double power = 0.5;

    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false, false);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();



        waitForStart();

        drive(5000);
    }
    private void drive(int ms) {


    }



}
