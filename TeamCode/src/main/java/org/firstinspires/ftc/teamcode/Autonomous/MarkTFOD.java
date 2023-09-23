package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.List;

public class MarkTFOD extends DriveMethods {
    public void runOpMode () {
        initVision(Variables.VisionProcessors.TFOD);

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            List<Recognition> recognitionList = tfod.getRecognitions();

            telemetry.addData("Objects dected #", recognitionList.size());

            for(Recognition recognition : recognitionList) {
                double x = (recognition.getLeft() + recognition.getRight())/2;
                double y = (recognition.getTop() + recognition.getBottom())/2;

                telemetry.addData("", "");
                telemetry.addData("Image", "%s (%.Of %% Conf.)",
                        recognition.getLabel(),
                        recognition.getConfidence() * 100
                );

                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f",
                        recognition.getWidth(),
                        recognition.getHeight()
                );
            }

            telemetry.update();

            sleep(50);
        }
    }
}
