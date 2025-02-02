package com.kalipsorobotics.test;



import com.kalipsorobotics.localization.KalmanFilter;
import com.kalipsorobotics.localization.OdometrySpark;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

@TeleOp(name = "KalmanFilterTest")
public class TestKalmanOdometry extends LinearOpMode {
    @Override
    public void runOpMode() {
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        KalmanFilter kalmanFilter = new KalmanFilter(0.8, 4);
        OdometrySpark odometrySpark = new OdometrySpark(otos);
        kalmanFilter.reset();
        waitForStart();

        while (opModeIsActive()) {
            Point noisyMeasurement = new Point(odometrySpark.sparkUpdateData().getX(),odometrySpark.sparkUpdateData().getY());

            // Filter the measurement
            Point filteredPoint = kalmanFilter.update(noisyMeasurement);
            telemetry.addLine("Noisy data");
            telemetry.addLine("x: " + noisyMeasurement.x + " y: " + noisyMeasurement.y);
            telemetry.addLine("filtered data");
            telemetry.addLine("x: " + filteredPoint.x + " y: " + filteredPoint.y);
            if (gamepad1.a) {
                kalmanFilter.reset();
                telemetry.addLine("odometry values reset");
            }
            telemetry.update();
        }
    }
}
