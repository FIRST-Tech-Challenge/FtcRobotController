package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;

@Autonomous(name = "BlueDetectTest")
public class BlueDetectTest extends InitializeCameras {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV);

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
