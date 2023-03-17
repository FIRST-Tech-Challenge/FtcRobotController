package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "StackGrabbingTest")
public class StackGrabbingTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        waitForStart();
        sleep(300);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(300);
        for(int i = 0; i <= 4; i++) {
            driveSlidesAutonomous((Constants.SLIDE_STACK_FOUR - 150)-(170*i));
            //sleep(500);
            centerConeStack(robotCameraPipeline);
            sleep(300);
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            sleep(300);
            driveSlidesAutonomous(Constants.SLIDE_LOW);
            //sleep(500);
            driveAutonomous(180, 62);
            sleep(300);
            turnToAngle(-90);
            //sleep(500);
            driveSlides(Constants.SLIDE_TOP);
            sleep(600);
            driveAutonomous(0, 2.3);
            sleep(300);
            driveSlides(Constants.SLIDE_HIGH);
            //sleep(500);
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            sleep(100);
            driveSlides(Constants.SLIDE_TOP);
            sleep(200);
            driveAutonomous(180, 4);
            sleep(200);
            driveSlidesAutonomous(Constants.SLIDE_BOTTOM);
            //sleep(500);
            turnToAngle(0);
            //sleep(500);
        }
    }
}