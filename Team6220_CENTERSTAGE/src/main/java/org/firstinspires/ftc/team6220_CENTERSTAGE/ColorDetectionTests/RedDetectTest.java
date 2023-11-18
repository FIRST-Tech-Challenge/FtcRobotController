package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetection;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;

@Autonomous(name = "RedDetectTest")
public class RedDetectTest extends LinearOpMode {

    // passing in makes it use redDetect so the camera can be accessed
    ColorDetection redDetect = new ColorDetection(this);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize camera and such, specifying color ranges in the process
        redDetect.init(Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV);

        telemetry.addLine("Waiting For Start");
        telemetry.addLine("Camera Dimensions: " + Constants.CAMERA_WIDTH + ", " + Constants.CAMERA_HEIGHT);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //Displays readings of detected object
            telemetry.addData("propPos: ", redDetect.returnZone());
            telemetry.addData("Contour XPos", redDetect.centerPosX);
            telemetry.addData("Contour YPos", redDetect.centerPosY);
            telemetry.update();
        }

    }
}
