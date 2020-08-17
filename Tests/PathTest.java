package org.firstinspires.ftc.teamcode.rework.Tests;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rework.AutoTools.PIDController;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

@TeleOp
public class PathTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        initRobot();

        ArrayList<Waypoint> path = new ArrayList<Waypoint>();

        path.add(new Waypoint(0,0));
        path.add(new Waypoint(0,60));
        path.add(new Waypoint(-60,60));
        path.add(new Waypoint(-60,0));

        robot.pathModule.path = path;

        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            telemetryUpdateTime();

            robot.telemetryDump.addData("x: ", robot.odometryModule.worldX);
            robot.telemetryDump.addData("y: ", robot.odometryModule.worldY);
            robot.telemetryDump.addData("heading: ", Math.toDegrees(robot.odometryModule.worldAngleRad));

            robot.telemetryDump.addData("xMovement: ", robot.drivetrainModule.xMovement);
            robot.telemetryDump.addData("yMovement: ", robot.drivetrainModule.yMovement);
            robot.telemetryDump.addData("turnMovement: ", robot.drivetrainModule.turnMovement);

            robot.telemetryDump.addData("xVel: ", robot.velocityModule.xVel);
            robot.telemetryDump.addData("yVel: ", robot.velocityModule.yVel);
            robot.telemetryDump.addData("angleVel: ", robot.velocityModule.angleVel);
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
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

