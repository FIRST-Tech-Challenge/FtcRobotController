package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Get Cube",group = "Test")
public class TestGetCube extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap,telemetry,this);

        waitForStart();

        robot.getCube();
    }
}
