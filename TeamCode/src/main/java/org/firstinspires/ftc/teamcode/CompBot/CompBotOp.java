package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Viridian Competition Teleop",group="CompBot")
public class CompBotOp extends OpMode {
    CompBotHW r = new CompBotHW();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        r.m.driveRobotCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

        r.intake.set((gamepad1.a?1:0) - (gamepad1.b?1:0));
        r.lift.set((gamepad1.x?1:0) - (gamepad1.y?1:0));
        r.spin.set(gamepad1.right_trigger-gamepad1.left_trigger);
        r.bucket.setPosition(gamepad1.left_bumper?1:0);
    }

    @Override
    public void stop() {
        r.m.stop();
    }
}
