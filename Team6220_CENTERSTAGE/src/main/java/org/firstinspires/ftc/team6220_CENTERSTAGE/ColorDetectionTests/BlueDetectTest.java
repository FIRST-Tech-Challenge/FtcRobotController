package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetection;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;

@Disabled
@Autonomous(name = "BlueDetectTest")
public class BlueDetectTest extends LinearOpMode {

    // passing in makes it use blueDetect so the camera can be accessed
    ColorDetection blueDetect = new ColorDetection(this);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize camera and such, specifying color ranges in the process
        blueDetect.init(Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV);

        telemetry.addLine("Waiting For Start");
        telemetry.addLine("Necessary Camera Dimensions: " + Constants.CAMERA_WIDTH + ", " + Constants.CAMERA_HEIGHT);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //Displays readings of detected object
            telemetry.addData("propPos: ", blueDetect.returnZone());
            telemetry.addData("Contour XPos", blueDetect.centerPosX);
            telemetry.addData("Contour YPos", blueDetect.centerPosY);
            telemetry.update();
        }

    }
}
