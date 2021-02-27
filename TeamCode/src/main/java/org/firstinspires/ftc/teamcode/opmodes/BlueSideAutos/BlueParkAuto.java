package org.firstinspires.ftc.teamcode.opmodes.BlueSideAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.misc.PathPoint;

import java.util.ArrayList;
@Disabled
@Autonomous(name = "Blue Park Auto", group = "opModes")
public class BlueParkAuto extends UpliftAuto {

    UpliftRobot robot;
    Odometry odom;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() {
        // last thing to do is write the current robot position to file
        robot.stopThreads();
        robot.writePositionToFiles();
    }
}
