package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "AutonomousTest")
public class AutonomousTest extends BaseAutonomous {

    public GrabberCameraPipeline grabberCameraPipeline;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        camera.setPipeline(grabberCameraPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(1000);

        driveSlidesAutonomous(Constants.SLIDE_TOP);

        centerJunctionTop(grabberCameraPipeline);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);
    }
}