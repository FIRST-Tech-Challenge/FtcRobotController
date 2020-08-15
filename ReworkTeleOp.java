package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rework.Robot.ReworkRobot;

@TeleOp
public class ReworkTeleOp extends LinearOpMode {
    ReworkRobot robot;

    public void runOpMode() {
        initRobot();

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            robot.getBulkData();

            updateDrivetrainStates();

            displayUpdateTime();
        }
    }

    private void initRobot() {
        robot = new ReworkRobot(hardwareMap, telemetry,this);

        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrain.setStates(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
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
