package org.firstinspires.ftc.teamcode.Debugging;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Drivetrain;


@TeleOp
public class DrivetrainOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private Drivetrain drivetrain;
    private GamepadEx controller;
    private Logger logger;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        drivetrain = new Drivetrain(hardware, controller, logger, true);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        drivetrain.update();
        drivetrain.command();

        drivetrain.log();

        logger.print();
    }

}
