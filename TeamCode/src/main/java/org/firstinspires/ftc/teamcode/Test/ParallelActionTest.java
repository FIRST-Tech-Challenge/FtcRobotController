package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

@Autonomous(name="ParallelActionTest", group="Test")
public class ParallelActionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
    ViperSlide viperSlide = new ViperSlide(this);
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Initialized", "");
            telemetry.update();
        }

        waitForStart();

    }
}
