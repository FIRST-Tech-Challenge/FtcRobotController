package org.firstinspires.ftc.teamcode.Debugging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Deposit;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Drivetrain;
@Config
@TeleOp
public class DepositOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private Logger logger;
    private Deposit deposit;
    private Drivetrain drivetrain;
    private GamepadEx controller;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        deposit = new Deposit(hardware, controller, logger);
        drivetrain = new Drivetrain(hardware, controller, logger);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit.setTargetState(Deposit.TargetState.transfer);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        drivetrain.update();
        deposit.update();


        if (controller.wasJustPressed(GamepadKeys.Button.A)){

            deposit.setTargetState(Deposit.TargetState.transfer);

        } else if (controller.wasJustPressed(GamepadKeys.Button.B)) {
            deposit.setTargetState(Deposit.TargetState.specIntake);

        } else if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            deposit.setTargetState(Deposit.TargetState.sampleDeposit);
        } else if (controller.wasJustPressed(GamepadKeys.Button.X)) {


            if (deposit.getTargetState() != Deposit.TargetState.specDepositReady) {
                deposit.setTargetState(Deposit.TargetState.specDepositReady);
            } else if (deposit.getTargetState() == Deposit.TargetState.specDepositReady) {
                deposit.setTargetState(Deposit.TargetState.specDepositClipped);
            }

        }

        drivetrain.command();
        deposit.command();

        drivetrain.log();
        deposit.log();

        logger.print();
    }
}
