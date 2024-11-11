package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpMode extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        TestingDriveTrain driveTrain = new TestingDriveTrain(hardwareMap);
        waitForStart();
        boolean flag = false;

        while (opModeIsActive()) {
            if (!flag) {
                driveTrain.move(6, 0, 0, telemetry);
                flag = true;
            }
//            Point pos = driveTrain.odometryFuse.CollectData();
//            double heading = driveTrain.otos.getPosition().h;
//            telemetry.addLine(String.format("x | currently at %f", pos.getX()));
//            telemetry.addLine(String.format("y | currently at %f", pos.getY()));
//            telemetry.addLine(String.format("h | currently at %f", heading));
//            telemetry.update();
        }

    }
}
