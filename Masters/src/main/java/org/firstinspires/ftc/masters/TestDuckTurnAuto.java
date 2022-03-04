package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
@Autonomous(name= "test duck speed auto")
public class TestDuckTurnAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive= new SampleMecanumDrive(hardwareMap, this, telemetry);

        waitForStart();

        sampleMecanumDrive.jevilTurnRedCarousel(3);
    }
}
