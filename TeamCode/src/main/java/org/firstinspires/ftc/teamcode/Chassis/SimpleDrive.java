package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SimpleDrive extends OpMode {

    DcMotor TL;
    DcMotor TR;
    DcMotor BL;
    DcMotor BR;

    @Override
    public void init() {
        TL = hardwareMap.dcMotor.get("TL");
        TR = hardwareMap.dcMotor.get("TR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
    }

    @Override
    public void loop() {
        TL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        TR.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        BL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        BR.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
    }
}
