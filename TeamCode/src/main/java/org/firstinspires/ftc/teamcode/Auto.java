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
        drive.driveToPosition(0, 50, 0.7, 2);
        drive.driveToPosition(0, 0, 0.7, 2);
    }

    @Override
    public void exit() throws InterruptedException {
        robot.writePositionToFiles();
        robot.stopThreads();
    }
}
