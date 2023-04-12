package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        for (int i = 4; i >= 0; i--) {
            //turn LED's off (for testing purposes)
            blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            //drive slides to stack position
            driveSlidesAutonomous(Constants.STACK_HEIGHTS[i]);
            //center on cone stack
            centerConeStack(robotCameraPipeline);
            //close grabber
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            //wait for grabber to close
            sleep(500);
            driveAutonomous(180, 0.1);
            //drive slides to stow position
            sleep(100);
            driveSlidesAutonomous(Constants.SLIDE_LOW);
            //drive backwards 34.5 inches
            driveAutonomous(180, 34.7);
            sleep(100);
            //turn towards junction
            turnToAngle(90);
            //drive slides up
            driveSlidesAutonomous(Constants.SLIDE_HIGH-10);
            sleep(100);
            //wait for slides to go all the way up
            //drive forward
            driveAutonomous(0, 2);
            sleep(100);
            //lower slides onto junction
            driveSlidesAutonomous(Constants.SLIDE_HIGH-300);
            //open the grabber
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            //wait for cone to drop
            sleep(100);
            //drive slides back up
            driveSlidesAutonomous(Constants.SLIDE_HIGH);
            //drive backwards
            driveAutonomous(180, 2);
            //turn back to 0 heading
            turnToAngle(0);
        }
    }
}