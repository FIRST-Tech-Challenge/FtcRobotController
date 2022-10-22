package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Power Play Sleeve Test", group = "competition")
public class DetectBeaconSleeveColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor color = null;
//        color = CV.sleevePipeline.color;


        waitForStart();


        while (opModeIsActive()) {

            color = CV.sleevePipeline.color;
            telemetry.addData("Position", color);
            telemetry.update();

        }


    }
}
