package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Deposit;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Claw;

@TeleOp
public class ClawOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private GamepadEx controller;
    private Logger logger;

    private Claw claw;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);

        claw = new Claw(hardware, logger);
        claw.setTargetPosition(DepositConstants.clawOpenPos);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();
        claw.update();

        if (controller.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){

            if (claw.getStatus() == Claw.Status.gripped) {
                claw.setTargetPosition(DepositConstants.clawOpenPos);
            } else if (claw.getStatus() == Claw.Status.released) {
                claw.setTargetPosition(DepositConstants.clawClosedPos);
            }

        }

        claw.command();

        claw.log();
        logger.print();
    }

}
