package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Bucket;

@TeleOp
public class BucketOpMode extends OpMode {

    private Hardware hardware =  new Hardware();
    private Logger logger;

    private Bucket bucket;
    private GamepadEx controller;

    private double offset = 0.00;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller  = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        bucket = new Bucket(hardware, logger);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();
        bucket.update();

        if (controller.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            offset += 0.01;
        } else if (controller.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            offset -= 0.01;
        }

        if (controller.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            bucket.setBucketPosition(IntakeConstants.bucketDownPosition + offset);
            bucket.setRollerPower(IntakeConstants.intakingPower);
            bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
        } else {
            bucket.setBucketPosition(IntakeConstants.bucketUpPosition + offset);
            bucket.setRollerPower(0.00);
            bucket.setGatePosition(IntakeConstants.gateOpenPosition);
        }

        bucket.command();

        logger.log("Op-Mode", "", Logger.LogLevels.production);
        logger.log("Offset", offset, Logger.LogLevels.production);
        bucket.log();

        logger.print();
    }
}
