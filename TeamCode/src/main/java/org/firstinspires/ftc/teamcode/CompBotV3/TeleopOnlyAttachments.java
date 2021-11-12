package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleopOnlyAttachments extends OpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
        r.lift.setPower((gamepad1.x?1:0) - (gamepad1.y?1:0));
        r.spin.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        r.bucket.setPower(gamepad1.left_bumper?-1:1);

    }

    @Override
    public void stop() {
        r.stop();
    }
}
