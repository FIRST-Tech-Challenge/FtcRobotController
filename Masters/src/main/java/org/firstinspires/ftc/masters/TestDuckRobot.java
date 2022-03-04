package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
@Disabled
@Autonomous (name="test find duck with robot")
public class TestDuckRobot extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);
        drive.openCVInnitShenanigans("blue");
        MultipleCameraCV.DuckDeterminationPipeline.DuckPosition freightPosition = drive.analyzeDuck();
        waitForStart();
      //  drive.CV.webcam.stopStreaming();

        drive.findDuckBlue();
    }
}
