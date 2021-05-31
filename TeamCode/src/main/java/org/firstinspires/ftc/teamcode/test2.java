package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class test2 extends OpMode {
    private DcMotor fl;
    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
    }

    @Override
    public void loop() {
        fl.setPower(-gamepad1.left_stick_y);
    }
}