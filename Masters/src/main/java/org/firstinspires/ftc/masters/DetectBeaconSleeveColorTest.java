package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DetectBeaconSleeveColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor color = null;
        color = CV.sleevePipeline.color;

        waitForStart();

        telemetry.addData("Position", color);
        telemetry.update();


    }
}
