package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Nothing", group = "CenterStage")
public class NothingAuto extends LinearOpMode {

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Doing nothing");
        telemetry.update();

        while (opModeIsActive()){}
    }

}
