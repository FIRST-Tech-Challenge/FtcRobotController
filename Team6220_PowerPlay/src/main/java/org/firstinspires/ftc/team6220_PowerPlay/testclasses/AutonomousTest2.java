package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.ConeDetection;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "AutonomousTest2", group = "Test")
public class AutonomousTest2 extends AprilTagDetect {
    int stackHeight = 4;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        int signal = detectAprilTag();

        driveInches(0, 1);
        sleep(500);
        driveInches(180, 1);
        sleep(500);
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);
        driveSlides(300);
        sleep(500);
        driveInches(0, 52);
        sleep(500);
        driveInches(270, 10);
        sleep(500);
        driveSlides(Constants.SLIDE_HIGH);
        sleep(500);
        driveInches(0, 2);
        sleep(500);
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        sleep(500);
        driveInches(180, 2);
        sleep(500);
        driveSlides(800);
        sleep(500);
        telemetry.addLine("Error happened here");
        telemetry.update();
        turnToAngle(90);
        driveInches(0, 52);
        detectGrab();
        //Robot centers itself on the cone
        while(coneDetectionPipeline.distance > 50){
            motorFR.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
            motorFL.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
            motorBR.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
            motorBL.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
        }
        //the robot moves until the cone is in range
        while(coneDetectionPipeline.coneSize > 3000){
            motorFR.setPower(0.25);
            motorFL.setPower(-0.25);
            motorBR.setPower(0.25);
            motorBL.setPower(-0.25);
        }
        //60 is the height of the cone, multiply it by stack height
        driveSlides(stackHeight*60);
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);
        driveSlides(Constants.SLIDE_TOP);
    }}
