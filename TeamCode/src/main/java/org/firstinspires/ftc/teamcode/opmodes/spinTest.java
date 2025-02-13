package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "spinTest")
public class spinTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor spinner  = hardwareMap.get(DcMotor.class, "spinner");
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                spinner.setPower(1.0);
            }
        }
    }
}
