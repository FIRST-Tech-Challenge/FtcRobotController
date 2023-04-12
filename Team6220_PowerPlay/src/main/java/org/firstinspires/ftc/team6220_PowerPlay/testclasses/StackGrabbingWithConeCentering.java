package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "untested garbage ??? nice :)", group = "Worlds Auto Tests")
public class StackGrabbingWithConeCentering extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //init cameras
        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        //turn LED's off (for testing purposes)
        blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        waitForStart();
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        //drop first cone on junction
        grabFromStackAndDepositOnJunction(1,0);
        //loop after first cone has been dropped
        grabFromStackAndDepositOnJunctionPlusConeCentering(4,0);
    }
}
