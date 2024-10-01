package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class StickToArmMotor extends OpMode {
    public DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "arm");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
    }
}
