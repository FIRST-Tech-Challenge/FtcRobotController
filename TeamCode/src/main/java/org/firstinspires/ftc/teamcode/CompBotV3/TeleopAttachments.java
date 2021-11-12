package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopAttachments extends OpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();
    double initialHeading;
    boolean headingReset = false;

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y, x = -1*gamepad1.left_stick_x, turn = gamepad1.right_stick_x;

        y *= (gamepad1.right_bumper?0.4:1);
        x *= (gamepad1.right_bumper?0.4:1);
        turn *= (gamepad1.right_bumper?0.4:1);

        if(Math.abs(y) > Math.abs(x)) {
            x = 0;
        } else {
            y = 0;
        }
        if (Math.abs(turn) < 0.1 && Math.abs(x) > 0 && Math.abs(y) > 0) {
            if(!headingReset) {
                initialHeading = r.imu.getHeading();
                headingReset = true;
            } else {
                double error = r.imu.getHeading() - initialHeading;
                r.fl.setPower(MathUtils.clamp(y + x - CompBotV3.corrCoeff*error,-1,1));
                r.fr.setPower(MathUtils.clamp(-(y - x + CompBotV3.corrCoeff*error),-1,1));
                r.bl.setPower(MathUtils.clamp(y - x - CompBotV3.corrCoeff*error,-1,1));
                r.br.setPower(MathUtils.clamp(-(y + x + CompBotV3.corrCoeff*error),-1,1));
            }
        } else {
            r.fl.setPower(MathUtils.clamp(y+x+turn ,-1,1));
            r.fr.setPower(MathUtils.clamp(-(y-x-turn),-1,1));
            r.bl.setPower(MathUtils.clamp(y-x+turn,-1,1));
            r.br.setPower(MathUtils.clamp(-(y+x-turn),-1,1));
        }

        r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
        r.lift.setPower((gamepad1.dpad_up?1:0) - (gamepad1.dpad_down?1:0));
        r.spin.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        r.bucket.setPower(gamepad1.left_bumper?-1:1);

    }

    @Override
    public void stop() {
        r.stop();
    }
}
