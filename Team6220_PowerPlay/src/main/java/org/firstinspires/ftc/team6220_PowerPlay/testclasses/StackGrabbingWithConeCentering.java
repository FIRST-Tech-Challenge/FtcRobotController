package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "0+5 with cone centering", group = "Worlds Auto Tests")
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
        grabFromStackAndDepositOnJunctionPlusConeCentering(5,-90);
        driveSlidesAutonomous(0);
    }
}
