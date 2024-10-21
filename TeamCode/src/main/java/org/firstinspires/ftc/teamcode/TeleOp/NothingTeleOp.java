package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class NothingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        telemetry.addData("What the sigma?", 1);
    }
}
