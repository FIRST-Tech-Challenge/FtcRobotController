package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "YARRRRRG")
public class Jarlsbergian_Teleop extends OpMode {

    JarlsCHasse drivetrain;
    Odometry_Info odo;


    @Override
    public void init() {
        drivetrain = new JarlsCHasse(hardwareMap);
        odo = new Odometry_Info(hardwareMap);
    }

    @Override
    public void loop() {

        odo.updateCurPos();
        drivetrain.GamepadInputs(90, gamepad1);

    }
}
