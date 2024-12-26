package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Deposit;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Drivetrain;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Intake;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;

@TeleOp
public class TeleOpV0 extends OpMode {
    private Hardware hardware = new Hardware();
    private Logger logger;
    private GamepadEx controller;

    private Deposit deposit;
    private Intake intake;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);

        drivetrain = new Drivetrain(hardware, controller, logger);
        deposit = new Deposit(hardware, controller, logger);
        intake = new Intake(hardware, controller, logger);

        deposit.setTargetState(Deposit.TargetState.transfer);
        intake.setTargetState(Intake.SystemState.Stowed);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        intake.update();
        deposit.update();
        drivetrain.update();

        // Intake Controls

        // Stow Intake |X|
        if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            intake.setTargetState(Intake.SystemState.Stowed);
        }

        // Deploy and Start Intaking toggle |B|
        if (controller.wasJustPressed(GamepadKeys.Button.B)) {

            if (intake.getTargetSystemState() == Intake.SystemState.Deployed) {
                intake.setTargetState(Intake.SystemState.Intaking);
            } else {
                intake.setTargetState(Intake.SystemState.Deployed);
            }

        }

        // Send deposit to transfer position |A|
        if (controller.wasJustPressed(GamepadKeys.Button.A)) {

            // If you the intake isnt stowed or trying to stow, or if the deposit is already at pre transfer or transfer, set target state to transfer
            // Written bug where you need to press X to go to transfer position twice to get there, fixable with objective & Robot FSM
            if ((intake.getCurrentSystemState() != Intake.SystemState.Stowed && intake.getTargetSystemState() != Intake.SystemState.Stowed) || deposit.getCurrentState() == Deposit.TargetState.preTransfer || deposit.getCurrentState() == Deposit.TargetState.transfer) {
                deposit.setTargetState(Deposit.TargetState.transfer);
            } else {
                deposit.setTargetState(Deposit.TargetState.preTransfer);
            }
        }

        // Managed going to heights |Y|
        if (controller.wasJustPressed(GamepadKeys.Button.A)) {

            // If sample is yellow
            if (intake.getLastSeenColor() == SampleDetector.SampleColor.yellow) {
                deposit.setTargetState(Deposit.TargetState.sampleDeposit);
            }

            // If sample is alliance colored
            if ((intake.getLastSeenColor() == SampleDetector.SampleColor.blue) || (intake.getLastSeenColor() == SampleDetector.SampleColor.red)) {

            if (deposit.getTargetState() != Deposit.TargetState.specIntake) {
                    deposit.setTargetState(Deposit.TargetState.specIntake);
                } else {
                deposit.setTargetState(Deposit.TargetState.specDepositReady);
            }

            }

        }

        // Manage Spec Intake and Clipped Pos |LB|
        if (controller.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {

            if (deposit.getTargetState() == Deposit.TargetState.specDepositReady) {
                deposit.setTargetState(Deposit.TargetState.specDepositClipped);
            } else if (deposit.getTargetState() == Deposit.TargetState.specDepositClipped) {
                deposit.setTargetState(Deposit.TargetState.specDepositReady);
            }

        }

        // If intake current state is stowed, target intake state is stowed, current deposit state is transfer, target deposit state is transfer, and intake has a sample, then grip claw
        if ((intake.getCurrentSystemState() == Intake.SystemState.Stowed && intake.getTargetSystemState() == Intake.SystemState.Stowed) && (deposit.getCurrentState() == Deposit.TargetState.transfer && deposit.getTargetState() == Deposit.TargetState.transfer) && intake.hasSample) {
            deposit.setTransfer(true);

        } else {
            deposit.setTransfer(false);
        }

        intake.command();
        deposit.command();
        drivetrain.command();


        intake.log();
        deposit.log();
        drivetrain.log();

        logger.print();
    }
}
