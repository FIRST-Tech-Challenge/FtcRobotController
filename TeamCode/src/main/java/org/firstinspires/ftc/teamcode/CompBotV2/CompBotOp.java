package org.firstinspires.ftc.teamcode.CompBotV2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBot.CompBotHW;

@TeleOp(name="Viridian Competition Teleop",group="CompBotV2")
@Disabled
public class CompBotOp extends OpMode {
    CompBotHWV2 r = new CompBotHWV2();

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
