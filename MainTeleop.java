package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rework.Robot.Robot;

@TeleOp
public class MainTeleop extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        initRobot();

        waitForStart();

        robot.startModules();


        while (opModeIsActive()) {
            robot.getBulkData();
            updateDrivetrainStates();
            displayUpdateTime();
            telemetry.addLine("stick values " + gamepad1.left_stick_y);
            telemetry.addLine("x: " + robot.odometry.getRobotPosition().getLocation().x);
            telemetry.addLine("y: " + robot.odometry.getRobotPosition().getLocation().y);
            telemetry.addLine("heading: " + Math.toDegrees(robot.odometry.getRobotPosition().getHeading()));
            telemetry.addLine("yLeft encoder: " + robot.odometry.yLeft.getCurrentPosition()*-1);
            telemetry.addLine("yRight encoder: " + robot.odometry.yRight.getCurrentPosition());
            telemetry.addLine("mecanum encoder: " + robot.odometry.mecanum.getCurrentPosition());

        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrain.setStates(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }


    long lastUpdateTime = SystemClock.elapsedRealtime();
    long currentTime;

    /**
     * Calculates and displays (in telemetry) the loop time of the while(isOpModeActive) loop.
     */
    private void displayUpdateTime() {
        currentTime = SystemClock.elapsedRealtime();

        telemetry.addData("TeleOp while loop update time: ", currentTime - lastUpdateTime);

        lastUpdateTime = currentTime;

        telemetry.update();
    }
}
