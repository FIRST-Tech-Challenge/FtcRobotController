package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "StackGrabbingAuto_Blue", group = "Test")
public class StackGrabbingAuto_Blueside extends ConeDetection {
    int stackHeight = 4;
    int[] lowerBlue = {42, 128, 114};
    int[] upperBlue = {168, 242, 255};
    @Override
    public void runOpMode() throws InterruptedException
    {
        ConeDetectionPipeline coneDetectionPipeline = new ConeDetectionPipeline();
        coneDetectionPipeline.setRanges(lowerBlue,upperBlue);
        initialize();
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        driveSlidesAutonomous(Constants.SLIDE_LOW);
        driveInches(0,40);
    }}














































































