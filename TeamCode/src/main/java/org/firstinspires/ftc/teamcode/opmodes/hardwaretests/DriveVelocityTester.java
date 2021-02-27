package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

@Autonomous(name = "Drive Velocity Tester", group = "Hardware Testers")
public class DriveVelocityTester extends UpliftAuto {
    UpliftRobot robot;
    DriveSubsystem driveSub;
    Odometry odom;
    DriveVelocityThread driveVelocityThread;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        driveSub = robot.driveSub;
        odom = robot.odometry;
        driveVelocityThread = new DriveVelocityThread();
        driveVelocityThread.t.start();
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        driveSub.driveToPosition(0, 60, 1, 0.5);
    }

    @Override
    public void exit() throws InterruptedException {
        driveVelocityThread.t = null;
    }

    private class DriveVelocityThread implements Runnable {

        Thread t;
        // drive Velocity is in inches/second
        double driveVelocity = -1;

        public DriveVelocityThread() {
            t = new Thread(this);
        }

        @Override
        public void run() {
            while(t != null) {
                double initialTime = System.currentTimeMillis();
                double initialY = robot.worldY;
                double initialX = robot.worldX;
                Utils.sleep(20);
                double deltaTime = System.currentTimeMillis() - initialTime;
                double deltaY = robot.worldY - initialY;
                double deltaX = robot.worldX - initialX;
                double deltaPos = Math.abs(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
                driveVelocity = deltaPos / (deltaTime / 1000);
                Log.i("Drive Velocity", driveVelocity + "");
            }
        }

    }
}
