package org.firstinspires.ftc.teamcode.Debugging;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.Controller;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;


@TeleOp
public class DrivetrainTestOpmode extends OpMode {
    Hardware hardware = new Hardware();
    Drivetrain drivetrain;
    Controller controller;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new Controller(gamepad1, Controller.xBox);
        drivetrain = new Drivetrain(hardware, controller);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        drivetrain.update();
        drivetrain.command();

        telemetry.addData("Pos: ", hardware.pinPoint.getPosition());
        telemetry.update();
    }

}
