package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionPipeline;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedDetectTest")
public class RedDetectTest extends DetectorFramework {
    OpenCvCamera robotCamera;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        ColorDetectionPipeline pipeline1 = new ColorDetectionPipeline();
        robotCamera.setPipeline(pipeline1);

        pipeline1.setRanges(Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("propPos: ", pipeline1.returnZone());
            telemetry.update();

            sleep(100);
        }

    }
}
