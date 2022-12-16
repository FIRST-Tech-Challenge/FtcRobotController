package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.ConeDetectionPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "StackGrabbingAuto", group = "Test")
public class AutonomousTest2 extends AprilTagDetect {
    int stackHeight = 4;
    int[] lowerBlue = {42, 128, 114};
    int[] upperBlue = {168, 242, 255};
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        ConeDetectionPipeline coneDetectionPipeline = new ConeDetectionPipeline();
        coneDetectionPipeline.setRanges(lowerBlue, upperBlue);
        int signal = detectAprilTag();
        //Move the cone to a grabbing position
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);
        //Raise slides
        sleep(500);
        //Drive forward to the tile next to the junction
        driveInches(0, 52);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        //Move right towards the junction
        driveInches(270, 7);
        sleep(500);
        sleep(500);
        //Drive towards junction
        driveInches(0, 1.5);
        sleep(500);
        //Drop cone
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        sleep(500);
        //Drive backwards
        driveInches(180, 4);
        sleep(500);
        //Raise slides so camera can detect
        driveSlidesAutonomous(800);
        sleep(500);
        telemetry.addLine("Error happened here");
        telemetry.update();
        //Turn left
        driveInches(90, 10);
        turnToAngle(90);
        driveSlidesAutonomous(400);
        driveInches(0, 35);
        //Robot centers itself on the cone
        detectGrab();
        driveSlides(60*stackHeight);
        //Strafe until robot centered
        while(Math.abs(coneDetectionPipeline.distance) > 50){
            motorFL.setPower(0.25*Math.signum(coneDetectionPipeline.distance));
            motorFR.setPower(0.25*Math.signum(coneDetectionPipeline.distance));
            motorBL.setPower(0.25*Math.signum(coneDetectionPipeline.distance));
            motorBR.setPower(0.25*Math.signum(coneDetectionPipeline.distance));
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);

        //the robot moves until the cone is in range
        while(coneDetectionPipeline.coneSize > 500){
            telemetry.addData("coneSize", coneDetectionPipeline.coneSize);
            telemetry.update();

        }
        driveSlidesAutonomous(0);
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

            //60 is the height of the cone, multiply it by stack height
    }}
