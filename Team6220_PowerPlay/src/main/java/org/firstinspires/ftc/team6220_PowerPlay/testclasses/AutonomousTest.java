package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "AutonomousTest")
public class AutonomousTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        grabberCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                grabberCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        grabberCamera.setPipeline(grabberCameraPipeline);

        waitForStart();

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(1000);

        driveSlidesAutonomous(Constants.SLIDE_HIGH);

        sleep(2000);

        centerJunctionTop(grabberCameraPipeline);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);
    }
}