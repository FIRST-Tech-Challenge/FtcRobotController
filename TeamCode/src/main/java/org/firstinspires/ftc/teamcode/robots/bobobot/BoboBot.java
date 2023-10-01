package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config("BoboGameVariables")
@TeleOp(name="BoboOpMode", group="Challenge")
public class BoboBot extends OpMode {
    Robot bobo;
    @Override
    public void init() {
        bobo = new Robot(telemetry, hardwareMap);
    }

    @Override
    public void loop() {
        bobo.driveTrain.mechanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
