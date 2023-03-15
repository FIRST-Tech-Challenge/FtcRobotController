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
        driveSlidesAutonomous(Constants.SLIDE_TOP);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(500);
        centerConeStack(robotCameraPipeline);
        sleep(500);
        driveSlidesAutonomous(Constants.SLIDE_STACK_FOUR);
        sleep(200);
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        sleep(200);
        turnToAngle(-90);
        sleep(200);
        driveAutonomous(-90,57);
        sleep(200);
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        sleep(200);
        driveAutonomous(0,5);
        sleep(200);
        driveGrabber(Constants.GRABBER_OPEN_POSITION);
        sleep(200);




    }
}
