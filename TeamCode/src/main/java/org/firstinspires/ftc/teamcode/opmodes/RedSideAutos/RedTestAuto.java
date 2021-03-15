package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.AutoTimeout;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

@Autonomous(name = "Red Test Auto", group = "opModes")
public class RedTestAuto extends UpliftAuto {
    UpliftRobot robot;
    DriveSubsystem drive;
    AutoTimeout autoTimeout;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        drive = robot.driveSub;
        autoTimeout = new AutoTimeout(robot);
        autoTimeout.enable();
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        robot.odometry.setOdometryPosition(0, 0, 0);
        drive.driveToPosition(0, 60, 0.5, 0);
    }

    @Override
    public void exit() throws InterruptedException {
        autoTimeout.stop();
        robot.stopThreads();
    }

}