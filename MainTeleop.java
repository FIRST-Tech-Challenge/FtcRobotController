package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

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
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrainModule.yMovement = -gamepad1.left_stick_y;
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

        robot.telemetryDump.addHeader("---TeleOp Update Speed---");
        robot.telemetryDump.addData("TeleOp while loop update time: ", (currentTime - lastUpdateTime));

        lastUpdateTime = currentTime;
    }
}