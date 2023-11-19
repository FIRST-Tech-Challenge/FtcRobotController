package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class NothingAuto extends LinearOpMode {

    public void runOpMode() {
        telemetry.addData("Currently", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Currently", "Doing nothing");
        telemetry.update();

        while (opModeIsActive()){}
    }

}
