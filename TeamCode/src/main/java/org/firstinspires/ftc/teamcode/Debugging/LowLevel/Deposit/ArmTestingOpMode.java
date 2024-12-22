package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Deposit;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.Arm;

@TeleOp
public class ArmTestingOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private GamepadEx controller;
    private Arm arm;

    private double targetPosition = 0;
    private double offset = 0.00;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        arm = new Arm(hardware);
        arm.setPosition(targetPosition);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        if (controller.wasJustPressed(GamepadKeys.Button.A)){
            targetPosition = DepositConstants.armRightTransferPos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.B)){
            targetPosition = DepositConstants.armRightSpecDepositPos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            targetPosition = DepositConstants.armRightSampleDepositPos;
        }   else if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            targetPosition = DepositConstants.armRightSpecIntakePos;
        } else if (controller.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            targetPosition = 0;
        }

        if (controller.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            offset += 0.01;
        } else if (controller.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            offset -= 0.01;
        }



        arm.setPosition(targetPosition + offset);

        telemetry.addData("Right Encoder Position", arm.getPosition());
        telemetry.addData("Offset: ", offset);
        telemetry.update();
        }

    }


