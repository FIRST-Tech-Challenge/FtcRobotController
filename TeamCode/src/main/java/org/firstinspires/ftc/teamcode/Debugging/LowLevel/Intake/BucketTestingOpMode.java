package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.Bucket;

@TeleOp
public class BucketTestingOpMode extends OpMode {

    private Hardware hardware =  new Hardware();

    private Bucket bucket;
    private GamepadEx controller;

    @Override
    public void init() {
        hardware.init(hardwareMap);

        controller  = new GamepadEx(gamepad1);
        bucket = new Bucket(hardware);
    }

    @Override
    public void loop() {
        controller.readButtons();

        if (controller.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            bucket.setBucketPosition(IntakeConstants.bucketDownPosition);
            bucket.setRollerPower(IntakeConstants.intakingPower);
            bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
        } else {
            bucket.setBucketPosition(IntakeConstants.bucketUpPosition);
            bucket.setRollerPower(0.00);
            bucket.setGatePosition(IntakeConstants.gateOpenPosition);
        }



    }
}
