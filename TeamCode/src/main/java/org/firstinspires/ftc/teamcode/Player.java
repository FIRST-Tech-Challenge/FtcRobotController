package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.player.Mecanum;

@TeleOp(name = "Driver OP")
public class Player extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum.initialize(this);

        waitForStart();

        while (opModeIsActive()){
            Mecanum.drive();

            telemetry.update();
        }
    }
}
