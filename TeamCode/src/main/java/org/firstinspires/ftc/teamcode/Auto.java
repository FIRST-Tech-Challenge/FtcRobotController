package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Arm;

@Autonomous(name = "Auto V0.0.1")
public class Auto  extends OpMode {

    private Hardware hardware = new Hardware();
    private Arm arm;
    private GamepadEx controller;
    private Logger logger;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        arm = new Arm(hardware, logger);

        arm.setPosition(DepositConstants.armRightSpecIntakePos);
        arm.command();
    }

    @Override
    public void loop() {
        arm.setPosition(0.4);
        arm.command();
    }
}