package org.firstinspires.ftc.teamcode.Debugging;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;


@TeleOp
public class DrivetrainTestOpmode extends OpMode {
    Hardware hardware = new Hardware();
    Drivetrain drivetrain;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        drivetrain = new Drivetrain(hardware, gamepad1, Drivetrain.Controller.xBox);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        hardware.pinPoint.update();
        drivetrain.update();

        telemetry.addData("Pos: ", hardware.pinPoint.getPosition());
        telemetry.update();
    }

}
