package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Innov8Teleop_Robot", group = "Robot")
public class Innov8Teleop_Robot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hera hera = new Hera(telemetry, hardwareMap, this);
        waitForStart();
        hera.teleop(gamepad1, gamepad2);
    }
}
