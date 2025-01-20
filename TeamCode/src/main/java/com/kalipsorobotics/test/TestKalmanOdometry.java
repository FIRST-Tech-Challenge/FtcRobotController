package com.kalipsorobotics.test;



import com.kalipsorobotics.localization.KalmanFilter;
import com.kalipsorobotics.localization.OdometrySpark;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.kalipsorobotics.localization.OdometrySpark;

import org.opencv.core.Point;

@TeleOp(name = "KalmanFilterTest")
public class TestKalmanOdometry extends LinearOpMode {
    @Override
    public void runOpMode() {
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        KalmanFilter kalmanFilter = new KalmanFilter(0.01, 1);
        OdometrySpark odometrySpark = new OdometrySpark(otos);
        waitForStart();

        while (opModeIsActive()) {
            // Example measurement (replace with real sensor data)
            Point noisyMeasurement = new Point(odometrySpark.sparkUpdateData().getX(),odometrySpark.sparkUpdateData().getY());

            // Filter the measurement
            Point filteredPoint = kalmanFilter.update(noisyMeasurement);

            // Display data
            telemetry.addLine("x: " + filteredPoint.x + "y: " + filteredPoint.y);
            telemetry.update();
        }
    }
}
