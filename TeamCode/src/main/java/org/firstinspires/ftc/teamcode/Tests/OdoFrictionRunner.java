package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OdoFrictionRunner extends LinearOpMode {

    private OdoFriction odoFriction;
    @Override
    public void runOpMode() {
        telemetry.clear();
        odoFriction = new OdoFriction(telemetry, hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            odoFriction.EndcoderTicks();
        }
    }

}
