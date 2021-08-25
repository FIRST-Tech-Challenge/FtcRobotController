package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
@TeleOp (name = "NitinOp", group = "Opmodes")
public class NitinTest extends UpliftTele {
    DcMotor tm;

    @Override
    public void initHardware() {
        tm = robot.testMotor;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() {
        tm.setPower(gamepad1.right_stick_y);

    }

    @Override
    public void exit() {

    }
}
