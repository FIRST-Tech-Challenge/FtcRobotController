package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Optional ON-SENSOR object recognition with a DFRobot HuskyLens.
 * Not used by the default webcam example. Remove @Disabled only if installed.
 */
@Disabled
@TeleOp(name = "Vision: HuskyLens AI (optional)", group = "Vision")
public class HuskyLensAiRecognitionOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        HuskyLens huskyLens = hardwareMap.get(
                HuskyLens.class, VisionConfig.HUSKYLENS_NAME);
        if (!huskyLens.knock()) {
            telemetry.addLine("HuskyLens is not responding. Check power and I2C wiring.");
            telemetry.update();
            return;
        }

        // Configure/train the recognition mode on the HuskyLens itself first.
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        telemetry.addLine("HuskyLens ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] objects = huskyLens.blocks();
            telemetry.addData("Recognized objects", objects.length);
            for (HuskyLens.Block object : objects) {
                // Object ID -> label mapping depends on the training in HuskyLens.
                telemetry.addData("Object ID " + object.id,
                        "x=%d y=%d width=%d height=%d",
                        object.x, object.y, object.width, object.height);
            }
            telemetry.update();
            sleep(75);
        }
    }
}
