package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name="Testing CV")
public class TestingCV extends LinearOpMode {

    FreightFrenzyComputerVisionShippingElementReversion CV;

    @Override
    public void runOpMode() throws InterruptedException {
        CV = new FreightFrenzyComputerVisionShippingElementReversion(hardwareMap, telemetry);
        FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
    }
    public FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.FreightPosition analyze() {
        return CV.pipeline.position;
    }

}

