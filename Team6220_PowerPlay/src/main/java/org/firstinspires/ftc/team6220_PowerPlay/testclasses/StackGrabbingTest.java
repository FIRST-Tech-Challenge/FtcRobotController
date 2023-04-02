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
        sleep(300);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(300);
        for (int i = 0; i <= 4; i++) {
            //turn LED's off (for testing purposes)
            blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            //drive slides to stack position
            driveSlidesAutonomous((Constants.SLIDE_STACK_FOUR)-(Constants.AUTONOMOUS_STACK_PER_CONE_OFFSET * i));
            //center on cone stack
            centerConeStack(robotCameraPipeline);
            sleep(300);
            //wait for grabber to close
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            sleep(600);
            //drive slides to stow position
            driveSlidesAutonomous(Constants.SLIDE_LOW);
            //drive backwards 34.5 inches
            driveAutonomous(180, 34.5);
            //turn towards junction
            turnToAngle(90);
            //drive slides up
            driveSlidesAutonomous(Constants.SLIDE_TOP);
            //wait for slides to go all the way up
            sleep(600);
            //drive forward
            driveAutonomous(0, 2);
            //center on junction
            centerJunctionTop(grabberCameraPipeline);
            sleep(100);
            //lower slides onto junction
            driveSlidesAutonomous(Constants.SLIDE_HIGH - 100);
            sleep(100);
            //open the grabber
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            //wait for cone to drop
            sleep(100);
            //drive slides back up
            driveSlidesAutonomous(Constants.SLIDE_TOP);
            sleep(200);
            //drive backwards
            driveAutonomous(180, 3);
            //turn back to 0 heading
            turnToAngle(0);
        }
    }
}