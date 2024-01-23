package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class HuskyLensTest extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                telemetry.addData("Objects Detected: ", blocks.length);
                telemetry.update();
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Object " + (i + 1) + ": ", blocks[i].toString());
                    telemetry.update();
                }
            } else {
                telemetry.addLine("No Object Detected");
                telemetry.update();
            }
            telemetry.update();
        }
    }
}
