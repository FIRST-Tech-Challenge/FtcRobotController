package org.firstinspires.ftc.team417_CENTERSTAGE.pidf;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

@TeleOp(name = "PID")
public class PIDFArmController extends BaseOpMode {
    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
        }
    }
}
