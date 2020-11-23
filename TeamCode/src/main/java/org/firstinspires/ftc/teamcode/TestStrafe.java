package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
class TestStrafe extends OpMode {
    DcMotor fr, fl, rr, rl;
    double power;
    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr_motor");
        fl = hardwareMap.dcMotor.get("fl_motor");
        rr = hardwareMap.dcMotor.get("rr_motor");
        rl = hardwareMap.dcMotor.get("rl_motor");
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