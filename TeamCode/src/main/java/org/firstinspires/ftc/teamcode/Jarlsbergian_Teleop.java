package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "YARRRRRG")
public class Jarlsbergian_Teleop extends OpMode {

    JarlsCHasse drivetrain;
    Odometry_Info odo;
    Telemetry.Line rotation;


    @Override
    public void init() {
        drivetrain = new JarlsCHasse(hardwareMap);
        odo = new Odometry_Info(hardwareMap);
        odo.resetEncoders();
    }

    @Override
    public void loop() {

        odo.updateCurPos();
        drivetrain.GamepadInputs( odo.cur0, gamepad1);

        telemetry.addData("Rotation", odo.cur0);
        telemetry.addData("odoLeft", odo.Cn1);
        telemetry.addData("odoRight", odo.Cn2);
        telemetry.addData("odoback", odo.Cn3);


    }
}
