package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "JunctionCenteringTest")
public class JunctionCenteringTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);

        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        waitForStart();

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        driveSlides(Constants.SLIDE_TOP);

        centerJunctionTop(grabberCameraPipeline);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);
    }
}
