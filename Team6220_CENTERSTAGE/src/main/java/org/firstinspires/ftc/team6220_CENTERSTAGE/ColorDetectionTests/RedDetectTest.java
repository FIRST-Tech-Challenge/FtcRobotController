package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionPipeline;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedDetectTest")
public class RedDetectTest extends InitializeCameras {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV);

        telemetry.addLine("Waiting For Start");
        telemetry.addLine("Camera Dimensions: " + Constants.CAMERA_WIDTH + ", " + Constants.CAMERA_HEIGHT);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("propPos: ", pipeline1.returnZone());
            telemetry.update();

            sleep(100);
        }

    }
}
