package com.kalipsorobotics.test;

import com.kalipsorobotics.localization.OdometrySpark;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestKalmanOdometry extends LinearOpMode {
    SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
    OdometrySpark odometry = new OdometrySpark(otos);
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            telemetry.addLine("" + odometry.sparkUpdateFiltered());
            telemetry.update();
        }
    }
}
