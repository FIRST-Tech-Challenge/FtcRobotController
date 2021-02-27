package org.firstinspires.ftc.teamcode.toolkit.core;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.misc.MathFunctions;

public abstract class UpliftTele extends LinearOpMode {

    public boolean forceStop;

    public boolean isStarted, isLooping, isFinished;

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void bodyLoop();

    public abstract void exit();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing", "Started");
        telemetry.update();

        initHardware();
        initAction();

        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();
        isStarted = true;

        telemetry.addData("Body", "Started");
        telemetry.update();

        while(opModeIsActive() && !isStopRequested() && !forceStop) {
            isLooping = true;
            bodyLoop();
        }

        telemetry.addData("Body", "Finished");
        telemetry.update();

        exit();
    }

    public void forceStop() {
        forceStop = true;
    }

    public void logData(UpliftRobot robot) {
        Log.i("LF Motor Power:", robot.leftFront.getPower() + "");
        Log.i("LB Motor Power:", robot.leftBack.getPower() + "");
        Log.i("RF Motor Power:", robot.rightFront.getPower() + "");
        Log.i("RB Motor Power:", robot.rightBack.getPower() + "");
    }

    public void displayFullTelemetry(UpliftRobot robot) {
        telemetry.addData("WorldX:\t", MathFunctions.truncate(robot.worldX));
        telemetry.addData("WorldY:\t", MathFunctions.truncate(robot.worldY));
        telemetry.addData("WorldOrientationAngle\t", robot.worldAngle);
        telemetry.addData("Left Encoder pos:\t", robot.odometry.getLeftTicks() / UpliftRobot.COUNTS_PER_INCH);
        telemetry.addData("Right Encoder pos:\t", robot.odometry.getRightTicks() / UpliftRobot.COUNTS_PER_INCH);
        telemetry.addData("Center Encoder pos:\t", robot.odometry.getCenterTicks() / UpliftRobot.COUNTS_PER_INCH);
        telemetry.addData("Slow Mode:\t", robot.slowMode);
        telemetry.addData("Shooting State\t",  robot.shootingState + "");
        telemetry.addData("Touch Sensor Value\t",  robot.digitalTouchTop.getState() + "");
        telemetry.update();
    }

}
