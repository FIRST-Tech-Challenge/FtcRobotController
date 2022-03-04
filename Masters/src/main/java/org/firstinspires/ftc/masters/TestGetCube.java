package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

//@Disabled
@Autonomous(name = "Test Get Cube",group = "Test")
public class TestGetCube extends LinearOpMode {
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap, this, telemetry);

        waitForStart();


            robot.lightSet();

            boolean found = robot.getCube(1400);

            if (found) {

                robot.pause(500);
            }




    }


}
