package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Parent's Teleop")
public class Yuvisberg extends OpMode {

    JarlsCHasse drivetrain;

    @Override
    public void init() {

        drivetrain = new JarlsCHasse(hardwareMap);

    }

    @Override
    public void loop() {

        drivetrain.GamepadInputs(0, gamepad1);

    }
}
