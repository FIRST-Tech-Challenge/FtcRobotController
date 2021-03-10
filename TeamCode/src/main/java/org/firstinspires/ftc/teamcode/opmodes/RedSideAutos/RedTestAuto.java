package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

@Autonomous(name = "Red Test Auto", group = "opModes")
public class RedTestAuto extends UpliftAuto {
    UpliftRobot robot;
    DriveSubsystem drive;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        drive = new DriveSubsystem(robot);
    }

    @Override
    public void initAction() {}

    @Override
    public void body() throws InterruptedException {
        drive.driveToPosition(0, 60, 1, 0);
        robot.safeSleep(2000);
        Log.i("Point 1", robot.worldX + "   " + robot.worldY + "   " + robot.worldAngle);
        drive.driveToPosition(-30, 30, 1, 0);
        robot.safeSleep(2000);
        Log.i("Point 1", robot.worldX + "   " + robot.worldY + "   " + robot.worldAngle);
        drive.driveToPosition(0, 60, 1, 180);
        robot.safeSleep(2000);
        Log.i("Point 1", robot.worldX + "   " + robot.worldY + "   " + robot.worldAngle);
        drive.moveForward(-1);
        robot.safeSleep(1000);
        drive.moveForward(1);
        drive.moveForward(0);
    }

    @Override
    public void exit() throws InterruptedException {
        robot.stopThreads();
    }

}