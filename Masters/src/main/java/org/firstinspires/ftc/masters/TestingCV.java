package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name="Testing CV")
public class TestingCV extends LinearOpMode {

    FreightFrenzyComputerVisionFindDuck CV;

    @Override
    public void runOpMode() throws InterruptedException {
        CV = new FreightFrenzyComputerVisionFindDuck(hardwareMap, telemetry);
        FreightFrenzyComputerVisionFindDuck.SkystoneDeterminationPipeline.DuckPosition duckLocation = null;

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            duckLocation = analyze();

            telemetry.addData("Position", duckLocation);
            telemetry.update();
        }
    }
    public FreightFrenzyComputerVisionFindDuck.SkystoneDeterminationPipeline.DuckPosition analyze() {
        return CV.pipeline.position;
    }

}

