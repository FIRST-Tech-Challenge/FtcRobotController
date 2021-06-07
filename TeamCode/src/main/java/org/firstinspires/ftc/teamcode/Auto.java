package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.functions.DriveFunctions;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

@Autonomous(name = "Auto", group = "OpModes")
public class Auto extends UpliftAuto {

    UpliftRobot robot;
    DriveFunctions drive;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        drive = robot.driveFunctions;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        drive.turnTo(240, 0.7, 1, DriveFunctions.CLOCKWISE);
        robot.safeSleep(3000);
        drive.turnTo(1, 0.7, 1, DriveFunctions.COUNTER_CLOCKWISE);
        robot.safeSleep(3000);
        drive.turnTo(179, 0.7, 1, DriveFunctions.QUICKEST_DIRECTION);
    }

    @Override
    public void exit() throws InterruptedException {
        robot.writePositionToFiles();
        robot.stopThreads();
    }
}
