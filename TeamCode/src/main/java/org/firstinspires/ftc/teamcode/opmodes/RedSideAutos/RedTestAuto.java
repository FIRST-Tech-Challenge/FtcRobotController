package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.AutoTimeout;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

@Autonomous(name = "Test Auto", group = "opModes")
public class RedTestAuto extends UpliftAuto {
    UpliftRobot robot;
    DriveSubsystem drive;
    AutoTimeout autoTimeout;
    IntakeSubsystem intake;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        drive = robot.driveSub;
        autoTimeout = new AutoTimeout(robot);
        autoTimeout.enable();
        intake = robot.intakeSub;
    }

    @Override
    public void initAction() {
        intake.initRoller();
    }

    @Override
    public void body() throws InterruptedException {
        robot.odometry.setOdometryPosition(0, 0, 0);
        drive.driveToPosition(0, 40, 1, 90);
        Log.i("Odometry", "REACHED FIRST POINT!!!");
        drive.driveToPosition(40, 40, 1, 180);
        Log.i("Odometry", "REACHED SECOND POINT!!!");
        drive.driveToPosition(40, 0 , 1, 180);
        Log.i("Odometry", "REACHED THIRD POINT!!!");
        drive.driveToPosition(0, 0, 1, 180);
        Log.i("Odometry", "REACHED LAST POINT!!!");
        drive.turnTo(0, 1, DriveSubsystem.QUICKEST_DIRECTION);
        Log.i("Odometry", "FINISHED TURNING!!!");
        drive.turnTo(0, 1, DriveSubsystem.QUICKEST_DIRECTION);
    }

    @Override
    public void exit() throws InterruptedException {
        autoTimeout.stop();
        robot.stopThreads();
    }

}