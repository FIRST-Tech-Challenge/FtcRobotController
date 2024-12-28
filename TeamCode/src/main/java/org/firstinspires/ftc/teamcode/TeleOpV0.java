package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Deposit;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Intake;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Robot;

import java.util.ArrayList;

@TeleOp
public class TeleOpV0 extends OpMode {
    private Hardware hardware = new Hardware();
    private Logger logger;
    private GamepadEx controller;

    private Robot robot;

    private ArrayList<SampleDetector.SampleColor> colors = new ArrayList<>();


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);

        robot  = new Robot(hardware, controller, logger);

        robot.setDepositDesiredState(Deposit.TargetState.transfer);
        robot.setIntakeDesiredState(Intake.SystemState.Stowed);

        colors.add(SampleDetector.SampleColor.blue);
        colors.add(SampleDetector.SampleColor.yellow);

        robot.setAcceptedSamples(colors);

    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        robot.update();



        // Deposit Controls
        // Send deposit to transfer position |A|
        if (controller.wasJustPressed(GamepadKeys.Button.A)) {

            robot.setDepositDesiredState(Deposit.TargetState.transfer);
            robot.releaseClaw();
        }

        // Go to deposit |Y|
        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {

            robot.goToDeposit();

        }

        // Manage Spec Intake and Clipped Pos |LB|
        if (controller.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.clipSpec();
        }

        robot.command();
        robot.log();
        logger.print();
    }
}
