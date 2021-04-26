package org.firstinspires.ftc.teamcode.opmodes.BlueSideAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;
@Disabled
@Autonomous(name = "Blue Test Auto", group = "opModes")
public class BlueTestAuto extends UpliftAuto {

    UpliftRobot robot;
    Odometry odom;
    RingDetector detector;
    VelocityData velocityData;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
        detector = robot.ringDetector;
        velocityData = robot.velocityData;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() {
        robot.stopThreads();
        robot.writePositionToFiles();
    }
}
