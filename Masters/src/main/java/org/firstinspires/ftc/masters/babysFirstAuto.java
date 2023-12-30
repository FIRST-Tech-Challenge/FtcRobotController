package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Baby's First Auto")
public class babysFirstAuto extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {

        CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        CenterStageComputerVisionPipelines.pos propPos = null;

        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = CV.propFind.position;
            telemetry.addData("Position", propPos);
        }

        robot.init();
        robot.forward(.3, .5);

    }

}
