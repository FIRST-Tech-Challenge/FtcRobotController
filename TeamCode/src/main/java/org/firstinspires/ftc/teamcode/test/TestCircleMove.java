package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.hardware.TestHardware;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Circle movement")
public class TestCircleMove extends TestHardware {



    @Override
    public void loop() {
        double power = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double angle = Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        telemetry.addData("x", gamepad1.right_stick_x);
        telemetry.addData("y", gamepad1.right_stick_y);
        telemetry.addData("raw angle", angle);
        power *= 0.5;
        double rotate = gamepad1.left_stick_x;
        rotate *= 0.25;
        telemetry.addData("power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("rotate", rotate);
        omniDrive.move(power, angle, rotate);
    }
}
