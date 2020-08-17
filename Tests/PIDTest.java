package org.firstinspires.ftc.teamcode.rework.Tests;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rework.AutoTools.PIDController;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.Robot;

@TeleOp
public class PIDTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        initRobot();
        PIDController pidController = new PIDController(0.05,0.0000001,0,robot);
        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            telemetryUpdateTime();
            pidController.PID(robot.odometryModule.worldY,56);
            robot.drivetrainModule.yMovement = pidController.scale;
            robot.telemetryDump.addData("x: ", robot.odometryModule.worldX);
            robot.telemetryDump.addData("y: ", robot.odometryModule.worldY);
            robot.telemetryDump.addData("heading: ", Math.toDegrees(robot.odometryModule.worldAngleRad));
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
    }

    long lastUpdateTime = SystemClock.elapsedRealtime();
    long currentTime;

    /**
     * Calculates and displays (in robot.telemetryDump) the loop time of the while(isOpModeActive) loop.
     */
    private void telemetryUpdateTime() {
        currentTime = SystemClock.elapsedRealtime();

        robot.telemetryDump.addData("TeleOp while loop update time: ", (currentTime - lastUpdateTime));

        lastUpdateTime = currentTime;
    }
}

