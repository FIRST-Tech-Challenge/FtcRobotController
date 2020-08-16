package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleop extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        initRobot();
        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            updateDrivetrainStates();

            telemetryUpdateTime();
            telemetry.addLine("x: " + robot.odometryModule.robotPosition.getLocation().x);
            telemetry.addLine("y: " + robot.odometryModule.robotPosition.getLocation().y);
            telemetry.addLine("heading: " + Math.toDegrees(robot.odometryModule.robotPosition.getHeading()));
            telemetry.addLine("yLeft encoder: " + robot.odometryModule.getyLeft().getCurrentPosition()*-1);
            telemetry.addLine("yRight encoder: " + robot.odometryModule.getyRight().getCurrentPosition());
            telemetry.addLine("mecanum encoder: " + robot.odometryModule.getMecanum().getCurrentPosition());

            telemetry.update();
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrainModule.yMovement = gamepad1.left_stick_y;
        robot.drivetrainModule.xMovement = gamepad1.left_stick_x;
        robot.drivetrainModule.turnMovement = gamepad1.right_stick_x;

        if (gamepad1.right_bumper){
            robot.drivetrainModule.isSlowMode = true;
        } else {
            robot.drivetrainModule.isSlowMode = false;
        }
    }

    long lastUpdateTime = SystemClock.elapsedRealtime();
    long currentTime;

    /**
     * Calculates and displays (in telemetry) the loop time of the while(isOpModeActive) loop.
     */
    private void telemetryUpdateTime() {
        currentTime = SystemClock.elapsedRealtime();

        telemetry.addData("TeleOp while loop update time: ", currentTime - lastUpdateTime);

        lastUpdateTime = currentTime;
    }
}
