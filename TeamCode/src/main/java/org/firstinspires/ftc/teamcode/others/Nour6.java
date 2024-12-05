package org.firstinspires.ftc.teamcode.others;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Nour6 extends OpMode {

    DcMotor left_motor;
    DcMotor right_motor;

    @Override
    public void init() {
        left_motor = hardwareMap.dcMotor.get("left_motor");
        right_motor = hardwareMap.dcMotor.get("right_motor");
        left_motor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "OpMode is starting");
    }

    @Override
    public void loop() {
        right_motor.setPower(gamepad1.right_stick_y);
        left_motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("Status", "OpMode is looping");
    }
}