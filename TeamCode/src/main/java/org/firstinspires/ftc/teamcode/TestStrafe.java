package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="StrafingTest", group="Pushbot")
public class TestStrafe extends OpMode {
    DcMotor fr, fl, rr, rl;
    double power;
    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("frontright");
        fl = hardwareMap.dcMotor.get("frontleft");
        rr = hardwareMap.dcMotor.get("backright");
        rl = hardwareMap.dcMotor.get("backleft");
    }
    @Override
    public void loop() {
        power = -gamepad2.left_stick_x;
        fl.setPower(power);
        fr.setPower(-power);
        rr.setPower(power);
        rl.setPower(-power);
    }
}