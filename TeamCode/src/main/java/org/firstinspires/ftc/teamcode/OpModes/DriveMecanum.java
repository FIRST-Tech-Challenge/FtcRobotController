package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp
public class DriveMecanum extends Control {

    @Override
    public void loop() {
        super.loop();
        double leftY = gamepad1.left_stick_y;// < .05 ? 0 : gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;// < .05 ? 0 : gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;
        telemetry.addData("driveMode", driveMode);
        telemetry.addData("Start Button", gamepad1.start);

        mecanumDrive(leftY, leftX, turn);
        resetIMU(gamepad1.back);
        switchDriveMode(gamepad1.start);

    }
}