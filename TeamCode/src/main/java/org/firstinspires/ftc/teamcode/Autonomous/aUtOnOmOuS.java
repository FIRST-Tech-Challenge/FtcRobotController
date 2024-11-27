package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Autonomous(name="Main")

public class aUtOnOmOuS extends LinearOpMode {

    Motors motor;
    IMU imu;
    Input input;

    public void runOpMode() throws InterruptedException {

        Cameras cam = new Cameras((hardwareMap));
        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, Just use for debugging.


        AprilTag aprilTag = new AprilTag();
        aprilTag.initAprilTag();
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()) {

            aprilTag.telemetryAprilTag();
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant RGB color.
            // Note: to take actions based on the detected color, simply use the colorSwatch in a comparison or switch.
            //  eg:
            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}
            PredominantColorProcessor.Result result = cam.getColorSensor().getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            telemetry.update();

            sleep(20);
        }


        motor = new Motors(hardwareMap);
        imu = new IMU(hardwareMap);
        input = new Input(hardwareMap);

        imu.SetYaw();

        int step = 0;

        waitForStart();
        //run once

//        while (opModeIsActive())
//        {
//            double yaw = imu.getAngle('y');
//
//            if(step == 0)
//            {
//                if (yaw > 88 && yaw < 92 ) {
//                    input.spin(50);
//                }
//                else {
//                    step++;
//                }
//            }
//            else
//            {
//                if (step == 1)
//                {
//
//                }
//            }
//
//            telemetry.addData("Step:", "step (%.2f),", step);
//            telemetry.addData("IMU YAW:", "yaw (%.2f),", yaw);
//
//            telemetry.update(); // telemtryy
//        }
    }
}
