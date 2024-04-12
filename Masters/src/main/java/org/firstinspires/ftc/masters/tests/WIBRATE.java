package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "WIBRATE")
public class WIBRATE extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                gamepad1.rumble(100);
            }
        }
    }

}
