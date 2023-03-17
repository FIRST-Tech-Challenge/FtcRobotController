package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "StackGrabbingTest")
public class StackGrabbingTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        waitForStart();
        sleep(500);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_STACK_FOUR);
        sleep(500);
        centerConeStack(robotCameraPipeline);
        sleep(500);
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_LOW);
        sleep(500);
        turnToAngle(-90);
        sleep(500);
        driveAutonomous(-90,57);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        sleep(500);
        driveAutonomous(0,5);
        sleep(500);
        driveGrabber(Constants.GRABBER_OPEN_POSITION);
        sleep(500);
        driveAutonomous(180,5);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_BOTTOM);




    }
}
