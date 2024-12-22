package org.firstinspires.ftc.teamcode.Debugging;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Deposit;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;

@TeleOp
public class DepositTestingOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private Deposit deposit;
    private Drivetrain drivetrain;
    private GamepadEx controller;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        deposit = new Deposit(hardware);
        drivetrain = new Drivetrain(hardware, controller);

    }

    @Override
    public void loop() {
        hardware.clearCache();
        drivetrain.update();
        deposit.update();
        controller.readButtons();

        if (controller.getButton(GamepadKeys.Button.A) && controller.wasJustPressed(GamepadKeys.Button.A)){

            deposit.goToTransfer();

        } else if (controller.getButton(GamepadKeys.Button.B) && controller.wasJustPressed(GamepadKeys.Button.B)) {
            deposit.goToSpecIntake();

        } else if (controller.getButton(GamepadKeys.Button.Y) && controller.wasJustPressed(GamepadKeys.Button.Y)) {
            deposit.goToSampleDeposit();
        } else if (controller.getButton(GamepadKeys.Button.X) && controller.wasJustPressed(GamepadKeys.Button.X)) {


            if (deposit.getTargetState() != Deposit.TargetState.specDepositReady) {
                deposit.goToSpecDepositReady();
            } else if (deposit.getTargetState() == Deposit.TargetState.specDepositReady) {
                deposit.goToSpecClipped();
            }

        }

        if (controller.getButton(GamepadKeys.Button.RIGHT_BUMPER) && controller.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

            if (deposit.getClawState() == Deposit.ClawState.released) {
                deposit.grip();
            } else {
                deposit.release();
            }
        }
        telemetry.addData("SlidePos: ", deposit.getSlidePos());
        drivetrain.command();
        deposit.command();

    }
}
