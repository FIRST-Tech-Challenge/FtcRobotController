package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "StackGrabbingAuto_Red", group = "Test")
public class StackGrabbingAuto_Redside extends ConeDetection {
    int stackHeight = 4;
    int[] lowerRed = {42, 128, 114};
    int[] upperRed = {168, 242, 255};

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        ConeDetectionPipeline coneDetectionPipeline = new ConeDetectionPipeline();
        coneDetectionPipeline.setRanges(lowerRed, upperRed);
        //Grab cone
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);
        sleep(500);
        //Drive forward to the tile next to the junction
        driveInches(0, 52);
        sleep(500);
        //Raise slides
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        //Move right towards the junction
        driveInches(270, 10);
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
        driveInches(0, 35);
        detectGrab();
        //Strafe until the robot is centered on the cone
        while (stackHeight >= 0) {
            while (Math.abs(coneDetectionPipeline.distance) > 50) {
                motorFL.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
                motorFR.setPower(-0.25 * Math.signum(coneDetectionPipeline.distance));
                motorBL.setPower(0.25 * Math.signum(coneDetectionPipeline.distance));
                motorBR.setPower(-0.25 * Math.signum(coneDetectionPipeline.distance));
            }
            //Drive slides to current stack height
            driveSlidesAutonomous(stackHeight * 60);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
            motorBR.setPower(0);
            //the robot moves until the cone is in range
            while (coneDetectionPipeline.coneSize > 100) {
                telemetry.addData("distance to cone", coneDetectionPipeline.distance);
            }
            driveSlidesAutonomous(0);
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        }
        driveSlidesAutonomous((stackHeight * 60) + 100);
        //60 is the height of the cone, multiply it by stack height
    }
}














































































