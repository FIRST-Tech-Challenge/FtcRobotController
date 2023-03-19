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
        for (int i = 0; i <= 4; i++) {
            driveSlidesAutonomous((Constants.SLIDE_STACK_FOUR - Constants.AUTONOMOUS_STACK_BASE_OFFSET)-(Constants.AUTONOMOUS_STACK_PER_CONE_OFFSET * i));
            centerConeStack(robotCameraPipeline);
            sleep(300);
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            sleep(300);
            driveSlidesAutonomous(Constants.SLIDE_LOW);
            driveAutonomous(180, 62);
            turnToAngle(-90);
            driveSlides(Constants.SLIDE_TOP);
            sleep(600);
            driveAutonomous(0, 2.3);
            driveSlides(Constants.SLIDE_HIGH);
            centerJunctionTop(grabberCameraPipeline);
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            sleep(100);
            driveSlides(Constants.SLIDE_TOP);
            sleep(200);
            driveAutonomous(180, 4);
            sleep(200);
            driveSlidesAutonomous(Constants.SLIDE_BOTTOM);
            turnToAngle(0);
        }
    }
}