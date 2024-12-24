package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Deposit;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Arm;

@TeleOp
public class ArmOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private Logger logger;
    private GamepadEx controller;
    private Arm arm;

    private double offset = 0.00;

    private double targetPosition = DepositConstants.armRightTransferPos;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        arm = new Arm(hardware, logger);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();
        arm.update();

        if (controller.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            offset += 0.01;
        } else if (controller.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            offset -= 0.01;
        }

        if (controller.wasJustPressed(GamepadKeys.Button.A)){
            targetPosition = DepositConstants.armRightTransferPos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.B)){
            targetPosition = DepositConstants.armRightSpecDepositPos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            targetPosition= DepositConstants.armRightSampleDepositPos;
        }   else if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            targetPosition = DepositConstants.armRightSpecIntakePos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            arm.setPosition(0);
        }

        arm.setPosition(targetPosition + offset);

        arm.command();

        logger.log("Offset", offset, Logger.LogLevels.debug);

        arm.log();
        logger.print();

        }

    }


