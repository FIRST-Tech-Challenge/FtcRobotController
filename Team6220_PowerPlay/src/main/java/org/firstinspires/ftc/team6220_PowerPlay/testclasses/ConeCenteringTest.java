package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ConeCenteringTest")
public class ConeCenteringTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driveSlidesAutonomous(Constants.SLIDE_TOP);

        robotCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);

        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        waitForStart();

        centerConeStack(robotCameraPipeline, 300);

        telemetry.addData("width", robotCameraPipeline.width);

        telemetry.update();

        sleep(3000);

        robotCamera.closeCameraDevice();
    }
}
